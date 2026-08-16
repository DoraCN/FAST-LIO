//! Laser mapping main loop. Ported from `fast_lio/src/laserMapping.cpp`.
//!
//! Owns the buffers, time synchronization, FOV-based map sliding, voxel
//! downsampling, the point-to-plane measurement model and the incremental map
//! update. Driven by an offline data source through `add_imu` / `add_lidar_*`
//! and `run_once`.

use std::collections::HashMap;
use std::collections::VecDeque;

use nalgebra::{DMatrix, DVector, SMatrix, UnitQuaternion, Vector3};

use crate::consts::{G_M_S2, INIT_TIME, LASER_POINT_COV, LIDAR_SP_LEN, NUM_MATCH_POINTS};
use crate::esekf::{DynShareData, EseKf};
use crate::ikdtree::{BoxPointType, KdTree};
use crate::imu_processing::ImuProcess;
use crate::math::manifold::{StateIkfom, S2_GRAV_LENGTH, S2_GRAV_TYP};
use crate::math::s2::S2;
use crate::math::so3::{skew, M3D, V3D};
use crate::preprocess::Preprocess;
use crate::types::{AviaMsg, ImuRaw, LidarType, MeasureGroup, PointCloud, PointType, StandardMsg, TimeUnit};

const MOV_THRESHOLD: f32 = 1.5;
const EPSS: f32 = 1e-6;

/// Configuration for the whole pipeline (mirrors the ROS parameters of the
/// C++ node plus the yaml config files).
#[derive(Clone, Debug)]
pub struct LioConfig {
    pub lidar_type: LidarType,
    pub feature_extract_enable: bool,
    pub point_filter_num: i32,
    pub blind: f64,
    pub n_scans: usize,
    pub scan_rate: i32,
    pub timestamp_unit: TimeUnit,
    pub filter_size_surf: f32,
    pub filter_size_map: f32,
    pub cube_len: f64,
    pub det_range: f32,
    pub fov_deg: f64,
    pub gyr_cov: f64,
    pub acc_cov: f64,
    pub b_gyr_cov: f64,
    pub b_acc_cov: f64,
    pub extrinsic_est_en: bool,
    pub time_sync_en: bool,
    pub time_offset_lidar_to_imu: f64,
    pub extrinsic_t: [f64; 3],
    pub extrinsic_r: [f64; 9],
    pub max_iteration: usize,
}

impl Default for LioConfig {
    fn default() -> Self {
        Self {
            lidar_type: LidarType::Avia,
            feature_extract_enable: false,
            point_filter_num: 2,
            blind: 0.01,
            n_scans: 16,
            scan_rate: 10,
            timestamp_unit: TimeUnit::Us,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            cube_len: 1000.0,
            det_range: 300.0,
            fov_deg: 180.0,
            gyr_cov: 0.1,
            acc_cov: 0.1,
            b_gyr_cov: 0.0001,
            b_acc_cov: 0.0001,
            extrinsic_est_en: true,
            time_sync_en: false,
            time_offset_lidar_to_imu: 0.0,
            extrinsic_t: [0.0, 0.0, 0.0],
            extrinsic_r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            max_iteration: 4,
        }
    }
}

/// Output of one processed frame.
#[derive(Clone, Debug)]
pub struct LioResult {
    pub time: f64,
    pub pos: V3D,
    /// quaternion (w, x, y, z)
    pub quat: [f64; 4],
    pub vel: V3D,
    pub bg: V3D,
    pub ba: V3D,
    pub map_points: i32,
    pub effct_feat_num: usize,
    pub res_mean: f64,
}

/// The full laser-inertial mapping front-end.
pub struct LaserMapping {
    pub preprocess: Preprocess,
    pub imu: ImuProcess,
    pub kf: EseKf,
    pub ikdtree: KdTree,

    // config
    filter_size_surf_min: f32,
    filter_size_map_min: f32,
    cube_len: f64,
    det_range: f32,
    extrinsic_est_en: bool,
    lidar_type: LidarType,

    // buffers
    time_buffer: VecDeque<f64>,
    lidar_buffer: VecDeque<PointCloud>,
    imu_buffer: VecDeque<ImuRaw>,
    /// Persistent measurement group reused by `sync_packages` (matches the
    /// C++ global `MeasureGroup Measures`).
    meas: MeasureGroup,
    last_timestamp_lidar: f64,
    last_timestamp_imu: f64,
    lidar_end_time: f64,
    lidar_mean_scantime: f64,
    scan_num: i32,
    lidar_pushed: bool,

    // state flags
    flg_first_scan: bool,
    first_lidar_time: f64,
    flg_ekf_inited: bool,

    // local map (FOV)
    local_map_points: BoxPointType,
    localmap_initialized: bool,
    cub_needrm: Vec<BoxPointType>,

    // per-frame working state
    feats_undistort: PointCloud,
    feats_down_body: PointCloud,
    feats_down_world: PointCloud,
    point_selected_surf: Vec<bool>,
    nearest_points: Vec<Vec<PointType>>,
    normvec: Vec<PointType>,
    laser_cloud_ori: Vec<PointType>,
    corr_normvect: Vec<PointType>,
    res_last: Vec<f32>,
    effct_feat_num: usize,
    total_residual: f64,
    res_mean_last: f64,
}

impl LaserMapping {
    pub fn new(cfg: &LioConfig) -> Self {
        let mut preprocess = Preprocess::new();
        preprocess.set(
            cfg.feature_extract_enable,
            cfg.lidar_type,
            cfg.blind,
            cfg.point_filter_num,
        );
        preprocess.n_scans = cfg.n_scans;
        preprocess.scan_rate = cfg.scan_rate;
        preprocess.time_unit = cfg.timestamp_unit;

        let mut imu = ImuProcess::new();
        let lidar_r = M3D::from_iterator(cfg.extrinsic_r.iter().copied());
        let lidar_t = V3D::new(cfg.extrinsic_t[0], cfg.extrinsic_t[1], cfg.extrinsic_t[2]);
        imu.set_extrinsic(&lidar_t, &lidar_r);
        imu.set_gyr_cov(&V3D::new(cfg.gyr_cov, cfg.gyr_cov, cfg.gyr_cov));
        imu.set_acc_cov(&V3D::new(cfg.acc_cov, cfg.acc_cov, cfg.acc_cov));
        imu.set_gyr_bias_cov(&V3D::new(cfg.b_gyr_cov, cfg.b_gyr_cov, cfg.b_gyr_cov));
        imu.set_acc_bias_cov(&V3D::new(cfg.b_acc_cov, cfg.b_acc_cov, cfg.b_acc_cov));
        imu.lidar_type = cfg.lidar_type;

        let mut kf = EseKf::new(StateIkfom::default(), SMatrix::<f64, 23, 23>::identity());
        kf.maximum_iter = cfg.max_iteration;

        let mut ikdtree = KdTree::new();
        ikdtree.set_downsample_param(cfg.filter_size_map);

        Self {
            preprocess,
            imu,
            kf,
            ikdtree,
            filter_size_surf_min: cfg.filter_size_surf,
            filter_size_map_min: cfg.filter_size_map,
            cube_len: cfg.cube_len,
            det_range: cfg.det_range,
            extrinsic_est_en: cfg.extrinsic_est_en,
            lidar_type: cfg.lidar_type,
            time_buffer: VecDeque::new(),
            lidar_buffer: VecDeque::new(),
            imu_buffer: VecDeque::new(),
            meas: MeasureGroup::default(),
            last_timestamp_lidar: 0.0,
            last_timestamp_imu: -1.0,
            lidar_end_time: 0.0,
            lidar_mean_scantime: 0.0,
            scan_num: 0,
            lidar_pushed: false,
            flg_first_scan: true,
            first_lidar_time: 0.0,
            flg_ekf_inited: false,
            local_map_points: BoxPointType::default(),
            localmap_initialized: false,
            cub_needrm: Vec::new(),
            feats_undistort: Vec::new(),
            feats_down_body: Vec::new(),
            feats_down_world: Vec::new(),
            point_selected_surf: Vec::new(),
            nearest_points: Vec::new(),
            normvec: Vec::new(),
            laser_cloud_ori: Vec::new(),
            corr_normvect: Vec::new(),
            res_last: Vec::new(),
            effct_feat_num: 0,
            total_residual: 0.0,
            res_mean_last: 0.05,
        }
    }

    /// Feed one IMU sample (equivalent to `imu_cbk`).
    pub fn add_imu(&mut self, imu: &ImuRaw) {
        let timestamp = imu.stamp - self.time_diff_offset();
        if timestamp < self.last_timestamp_imu {
            self.imu_buffer.clear();
        }
        self.last_timestamp_imu = timestamp;
        let mut m = *imu;
        m.stamp = timestamp;
        self.imu_buffer.push_back(m);
    }

    fn time_diff_offset(&self) -> f64 {
        // offline driver keeps lidar & imu already synchronized; 0 offset
        0.0
    }

    /// Feed one Livox Avia frame (preprocess + buffer).
    pub fn add_lidar_avia(&mut self, msg: &AviaMsg) {
        if msg.stamp < self.last_timestamp_lidar {
            self.lidar_buffer.clear();
            self.time_buffer.clear();
        }
        self.last_timestamp_lidar = msg.stamp;
        let cloud = self.preprocess.process_avia(msg);
        self.lidar_buffer.push_back(cloud);
        self.time_buffer.push_back(msg.stamp);
    }

    /// Feed one standard (velodyne / ouster / marsim) frame.
    pub fn add_lidar_standard(&mut self, msg: &StandardMsg) {
        if msg.stamp < self.last_timestamp_lidar {
            self.lidar_buffer.clear();
            self.time_buffer.clear();
        }
        self.last_timestamp_lidar = msg.stamp;
        let cloud = self.preprocess.process_standard(msg);
        self.lidar_buffer.push_back(cloud);
        self.time_buffer.push_back(msg.stamp);
    }

    /// True when a synchronized measurement group is ready.
    pub fn has_data(&self) -> bool {
        !self.lidar_buffer.is_empty() && !self.imu_buffer.is_empty()
    }

    /// Process one synchronized frame. Returns `Some(result)` on success.
    pub fn run_once(&mut self) -> Option<LioResult> {
        if !self.sync_packages() {
            return None;
        }
        // snapshot the synchronized group (owned) so no borrow of self is held
        // while calling methods below
        let meas = MeasureGroup {
            lidar_beg_time: self.meas.lidar_beg_time,
            lidar_end_time: self.meas.lidar_end_time,
            lidar: self.meas.lidar.clone(),
            imu: self.meas.imu.clone(),
        };
        if self.flg_first_scan {
            self.first_lidar_time = meas.lidar_beg_time;
            self.imu.first_lidar_time = self.first_lidar_time;
            self.flg_first_scan = false;
            return None;
        }

        // IMU propagation + undistortion
        self.imu.process(&meas, &mut self.kf, &mut self.feats_undistort);
        let mut state_point = self.kf.get_x().clone();
        if self.feats_undistort.is_empty() {
            return None;
        }

        self.flg_ekf_inited = (meas.lidar_beg_time - self.first_lidar_time) >= INIT_TIME;
        let pos_lid = state_point.pos + state_point.rot * state_point.offset_t_l_i;
        self.lasermap_fov_segment(&pos_lid);

        // downsample the scan
        let leaf = self.filter_size_surf_min;
        self.feats_down_body = voxel_downsample(&self.feats_undistort, leaf);
        let feats_down_size = self.feats_down_body.len();

        // initialize the map kd-tree on the first usable scan
        if self.ikdtree.root.is_none() {
            if feats_down_size > 5 {
                self.feats_down_world.clear();
                for p in &self.feats_down_body {
                    let mut pw = PointType::default();
                    self.point_body_to_world(p, &mut pw, &state_point);
                    self.feats_down_world.push(pw);
                }
                self.ikdtree.build(self.feats_down_world.clone());
            }
            return None;
        }

        if feats_down_size < 5 {
            return None;
        }

        self.point_selected_surf.resize(feats_down_size, false);
        self.nearest_points.clear();
        self.nearest_points.resize(feats_down_size, Vec::new());
        self.res_last.resize(feats_down_size, 0.0);
        self.normvec.resize(feats_down_size, PointType::default());
        self.feats_down_world.resize(feats_down_size, PointType::default());
        self.laser_cloud_ori.clear();
        self.corr_normvect.clear();

        // iterated EKF update with the point-to-plane measurement model
        let mut solve_h_time = 0.0;
        {
            let Self {
                kf,
                feats_down_body,
                feats_down_world,
                nearest_points,
                point_selected_surf,
                normvec,
                laser_cloud_ori,
                corr_normvect,
                res_last,
                effct_feat_num,
                total_residual,
                res_mean_last,
                ikdtree,
                extrinsic_est_en,
                ..
            } = self;

            kf.update_iterated_dyn_share_modified(
                LASER_POINT_COV,
                &mut solve_h_time,
                |x, share| {
                    h_share_model(
                        x,
                        share,
                        feats_down_body,
                        feats_down_world,
                        nearest_points,
                        point_selected_surf,
                        normvec,
                        laser_cloud_ori,
                        corr_normvect,
                        res_last,
                        effct_feat_num,
                        total_residual,
                        res_mean_last,
                        ikdtree,
                        *extrinsic_est_en,
                    );
                },
            );
        }

        state_point = self.kf.get_x().clone();

        // add the feature points to the map
        self.map_incremental();

        // build result
        let q = state_point.rot.as_ref();
        Some(LioResult {
            time: meas.lidar_beg_time,
            pos: state_point.pos,
            quat: [q.w, q.i, q.j, q.k],
            vel: state_point.vel,
            bg: state_point.bg,
            ba: state_point.ba,
            map_points: self.ikdtree.validnum(),
            effct_feat_num: self.effct_feat_num,
            res_mean: self.res_mean_last,
        })
    }

    /// `sync_packages`: pull one lidar scan and the IMU samples covering it.
    /// The `MeasureGroup` persists in `self.meas` so that the lidar frame set
    /// while waiting for IMU is retained (matches the C++ global).
    fn sync_packages(&mut self) -> bool {
        if self.lidar_buffer.is_empty() || self.imu_buffer.is_empty() {
            return false;
        }
        if !self.lidar_pushed {
            self.meas.lidar = self.lidar_buffer.front().cloned().unwrap_or_default();
            self.meas.lidar_beg_time = *self.time_buffer.front().unwrap_or(&0.0);

            let short = self.meas.lidar.len() <= 1
                || (self.meas.lidar.last().map(|p| p.curvature as f64 / 1000.0)).unwrap_or(0.0)
                    < 0.5 * self.lidar_mean_scantime;
            if short {
                self.lidar_end_time = self.meas.lidar_beg_time + self.lidar_mean_scantime;
            } else {
                self.scan_num += 1;
                self.lidar_end_time = self.meas.lidar_beg_time
                    + self.meas.lidar.last().unwrap().curvature as f64 / 1000.0;
                self.lidar_mean_scantime +=
                    (self.meas.lidar.last().unwrap().curvature as f64 / 1000.0
                        - self.lidar_mean_scantime)
                        / self.scan_num as f64;
            }
            if self.lidar_type == LidarType::Marsim {
                self.lidar_end_time = self.meas.lidar_beg_time;
            }
            self.meas.lidar_end_time = self.lidar_end_time;
            self.lidar_pushed = true;
        }
        if self.last_timestamp_imu < self.lidar_end_time {
            return false;
        }
        self.meas.imu.clear();
        while let Some(front) = self.imu_buffer.front().copied() {
            if front.stamp > self.lidar_end_time {
                break;
            }
            self.meas.imu.push(front);
            self.imu_buffer.pop_front();
        }
        self.lidar_buffer.pop_front();
        self.time_buffer.pop_front();
        self.lidar_pushed = false;
        true
    }

    /// `lasermap_fov_segment`: slide the local map box and delete out-of-range boxes.
    #[allow(clippy::needless_range_loop)]
    fn lasermap_fov_segment(&mut self, pos_lid: &V3D) {
        self.cub_needrm.clear();
        if !self.localmap_initialized {
            for i in 0..3 {
                self.local_map_points.vertex_min[i] = (pos_lid[i] - self.cube_len / 2.0) as f32;
                self.local_map_points.vertex_max[i] = (pos_lid[i] + self.cube_len / 2.0) as f32;
            }
            self.localmap_initialized = true;
            return;
        }
        let mut dist_to_map_edge = [[0.0f32; 2]; 3];
        let mut need_move = false;
        for i in 0..3 {
            dist_to_map_edge[i][0] =
                (pos_lid[i] - self.local_map_points.vertex_min[i] as f64).abs() as f32;
            dist_to_map_edge[i][1] =
                (pos_lid[i] - self.local_map_points.vertex_max[i] as f64).abs() as f32;
            if dist_to_map_edge[i][0] <= MOV_THRESHOLD * self.det_range
                || dist_to_map_edge[i][1] <= MOV_THRESHOLD * self.det_range
            {
                need_move = true;
            }
        }
        if !need_move {
            return;
        }
        let mut new_local = self.local_map_points;
        let mov_dist = ((self.cube_len as f32 - 2.0 * MOV_THRESHOLD * self.det_range) * 0.5 * 0.9)
            .max(self.det_range * (MOV_THRESHOLD - 1.0));
        for i in 0..3 {
            let mut tmp = self.local_map_points;
            if dist_to_map_edge[i][0] <= MOV_THRESHOLD * self.det_range {
                new_local.vertex_max[i] -= mov_dist;
                new_local.vertex_min[i] -= mov_dist;
                tmp.vertex_min[i] = self.local_map_points.vertex_max[i] - mov_dist;
                self.cub_needrm.push(tmp);
            } else if dist_to_map_edge[i][1] <= MOV_THRESHOLD * self.det_range {
                new_local.vertex_max[i] += mov_dist;
                new_local.vertex_min[i] += mov_dist;
                tmp.vertex_max[i] = self.local_map_points.vertex_min[i] + mov_dist;
                self.cub_needrm.push(tmp);
            }
        }
        self.local_map_points = new_local;
        if !self.cub_needrm.is_empty() {
            self.ikdtree.delete_point_boxes(&self.cub_needrm);
        }
    }

    fn point_body_to_world(&self, pi: &PointType, po: &mut PointType, s: &StateIkfom) {
        let p_body = V3D::new(pi.x as f64, pi.y as f64, pi.z as f64);
        let p_global = s.rot * (s.offset_r_l_i * p_body + s.offset_t_l_i) + s.pos;
        po.x = p_global[0] as f32;
        po.y = p_global[1] as f32;
        po.z = p_global[2] as f32;
        po.intensity = pi.intensity;
    }

    /// `map_incremental`: add the current scan's world points to the map kd-tree.
    #[allow(clippy::field_reassign_with_default)]
    fn map_incremental(&mut self) {
        let feats_down_size = self.feats_down_body.len();
        let mut point_to_add: Vec<PointType> = Vec::with_capacity(feats_down_size);
        let mut point_no_need_downsample: Vec<PointType> = Vec::with_capacity(feats_down_size);
        let state_point = self.kf.get_x().clone();

        for i in 0..feats_down_size {
            let pi = self.feats_down_body[i];
            let mut pw = PointType::default();
            self.point_body_to_world(&pi, &mut pw, &state_point);
            self.feats_down_world[i] = pw;

            if !self.nearest_points[i].is_empty() && self.flg_ekf_inited {
                let points_near = self.nearest_points[i].clone();
                let mut need_add = true;
                let mut mid_point = PointType::default();
                mid_point.x = (pw.x / self.filter_size_map_min).floor() * self.filter_size_map_min
                    + 0.5 * self.filter_size_map_min;
                mid_point.y = (pw.y / self.filter_size_map_min).floor() * self.filter_size_map_min
                    + 0.5 * self.filter_size_map_min;
                mid_point.z = (pw.z / self.filter_size_map_min).floor() * self.filter_size_map_min
                    + 0.5 * self.filter_size_map_min;
                let dist = calc_dist(&pw, &mid_point);
                if (points_near[0].x - mid_point.x).abs() > 0.5 * self.filter_size_map_min
                    && (points_near[0].y - mid_point.y).abs() > 0.5 * self.filter_size_map_min
                    && (points_near[0].z - mid_point.z).abs() > 0.5 * self.filter_size_map_min
                {
                    point_no_need_downsample.push(pw);
                    continue;
                }
                for readd_i in 0..NUM_MATCH_POINTS {
                    if points_near.len() < NUM_MATCH_POINTS {
                        break;
                    }
                    if calc_dist(&points_near[readd_i], &mid_point) < dist {
                        need_add = false;
                        break;
                    }
                }
                if need_add {
                    point_to_add.push(pw);
                }
            } else {
                point_to_add.push(pw);
            }
        }
        self.ikdtree.add_points(&mut point_to_add, true);
        self.ikdtree.add_points(&mut point_no_need_downsample, false);
    }
}

/// Voxel-grid downsampling (replaces `pcl::VoxelGrid`).
fn voxel_downsample(input: &[PointType], leaf: f32) -> Vec<PointType> {
    if leaf <= 0.0 {
        return input.to_vec();
    }
    #[derive(Default)]
    struct Acc {
        x: f32,
        y: f32,
        z: f32,
        intensity: f32,
        curvature: f32,
        count: u32,
    }
    let mut map: HashMap<(i32, i32, i32), Acc> = HashMap::with_capacity(input.len() / 4 + 1);
    for p in input {
        let key = (
            (p.x / leaf).floor() as i32,
            (p.y / leaf).floor() as i32,
            (p.z / leaf).floor() as i32,
        );
        let e = map.entry(key).or_default();
        e.x += p.x;
        e.y += p.y;
        e.z += p.z;
        e.intensity += p.intensity;
        e.curvature += p.curvature;
        e.count += 1;
    }
    map.into_values()
        .map(|e| {
            let n = e.count as f32;
            PointType {
                x: e.x / n,
                y: e.y / n,
                z: e.z / n,
                intensity: e.intensity / n,
                curvature: e.curvature / n,
                ..Default::default()
            }
        })
        .collect()
}

fn calc_dist(a: &PointType, b: &PointType) -> f32 {
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)
}

/// Plane fit via least squares on up to `NUM_MATCH_POINTS` neighbours
/// (`esti_plane` in common_lib.h). Returns false if the plane residual exceeds
/// the threshold.
fn esti_plane(pca_result: &mut [f32; 4], point: &[PointType], threshold: f32) -> bool {
    let a = DMatrix::<f32>::from_fn(NUM_MATCH_POINTS, 3, |r, c| match c {
        0 => point[r].x,
        1 => point[r].y,
        _ => point[r].z,
    });
    let b = DVector::<f32>::from_element(NUM_MATCH_POINTS, -1.0);
    let sol = a
        .svd(true, true)
        .solve(&b, 1e-6)
        .unwrap_or_else(|_| DVector::zeros(3));
    let n = sol.norm();
    if n < 1e-9 {
        return false;
    }
    pca_result[0] = sol[0] / n;
    pca_result[1] = sol[1] / n;
    pca_result[2] = sol[2] / n;
    pca_result[3] = 1.0 / n;
    for p in point {
        let d = (pca_result[0] * p.x + pca_result[1] * p.y + pca_result[2] * p.z + pca_result[3])
            .abs();
        if d > threshold {
            return false;
        }
    }
    true
}

/// The point-to-plane measurement model (`h_share_model`).
#[allow(clippy::too_many_arguments)]
fn h_share_model(
    s: &StateIkfom,
    share: &mut DynShareData,
    feats_down_body: &[PointType],
    feats_down_world: &mut [PointType],
    nearest_points: &mut [Vec<PointType>],
    point_selected_surf: &mut [bool],
    normvec: &mut [PointType],
    laser_cloud_ori: &mut Vec<PointType>,
    corr_normvect: &mut Vec<PointType>,
    res_last: &mut [f32],
    effct_feat_num: &mut usize,
    total_residual: &mut f64,
    res_mean_last: &mut f64,
    ikdtree: &mut KdTree,
    extrinsic_est_en: bool,
) {
    laser_cloud_ori.clear();
    corr_normvect.clear();
    *total_residual = 0.0;

    let feats_down_size = feats_down_body.len();
    for i in 0..feats_down_size {
        let point_body = feats_down_body[i];
        let p_body = V3D::new(point_body.x as f64, point_body.y as f64, point_body.z as f64);
        let p_global = s.rot * (s.offset_r_l_i * p_body + s.offset_t_l_i) + s.pos;
        let pw = &mut feats_down_world[i];
        pw.x = p_global[0] as f32;
        pw.y = p_global[1] as f32;
        pw.z = p_global[2] as f32;
        pw.intensity = point_body.intensity;

        let mut point_search_sq_dis = vec![0.0f32; NUM_MATCH_POINTS];
        let points_near = &mut nearest_points[i];
        if share.converge {
            ikdtree.nearest_search(
                pw,
                NUM_MATCH_POINTS as i32,
                points_near,
                &mut point_search_sq_dis,
                f32::INFINITY,
            );
            point_selected_surf[i] = if points_near.len() < NUM_MATCH_POINTS {
                false
            } else {
                point_search_sq_dis[NUM_MATCH_POINTS - 1] <= 5.0
            };
        }
        if !point_selected_surf[i] {
            continue;
        }

        let mut pabcd = [0.0f32; 4];
        point_selected_surf[i] = false;
        if esti_plane(&mut pabcd, points_near, 0.1) {
            let pd2 = pabcd[0] * pw.x + pabcd[1] * pw.y + pabcd[2] * pw.z + pabcd[3];
            let s_score = 1.0 - 0.9 * pd2.abs() / (p_body.norm().sqrt() as f32).max(1e-6);
            if s_score > 0.9 {
                point_selected_surf[i] = true;
                normvec[i].x = pabcd[0];
                normvec[i].y = pabcd[1];
                normvec[i].z = pabcd[2];
                normvec[i].intensity = pd2;
                res_last[i] = pd2.abs();
            }
        }
    }

    *effct_feat_num = 0;
    for i in 0..feats_down_size {
        if point_selected_surf[i] {
            laser_cloud_ori.push(feats_down_body[i]);
            corr_normvect.push(normvec[i]);
            *total_residual += res_last[i] as f64;
            *effct_feat_num += 1;
        }
    }

    if *effct_feat_num < 1 {
        share.valid = false;
        return;
    }
    *res_mean_last = *total_residual / *effct_feat_num as f64;

    let mut h_x = DMatrix::<f64>::zeros(*effct_feat_num, 12);
    let mut h = DVector::<f64>::zeros(*effct_feat_num);

    for i in 0..*effct_feat_num {
        let laser_p = laser_cloud_ori[i];
        let point_this_be = V3D::new(laser_p.x as f64, laser_p.y as f64, laser_p.z as f64);
        let point_be_crossmat = skew(&point_this_be);
        let point_this = s.offset_r_l_i * point_this_be + s.offset_t_l_i;
        let point_crossmat = skew(&point_this);

        let norm_p = corr_normvect[i];
        let norm_vec = V3D::new(norm_p.x as f64, norm_p.y as f64, norm_p.z as f64);

        let c = s.rot.conjugate() * norm_vec;
        let a = point_crossmat * c;
        // h_x columns: [pos(0-2), rot(3-5), offsetR(6-8), offsetT(9-11)]
        h_x[(i, 0)] = norm_p.x as f64;
        h_x[(i, 1)] = norm_p.y as f64;
        h_x[(i, 2)] = norm_p.z as f64;
        h_x[(i, 3)] = a[0];
        h_x[(i, 4)] = a[1];
        h_x[(i, 5)] = a[2];
        if extrinsic_est_en {
            let b = point_be_crossmat * (s.offset_r_l_i.conjugate() * c);
            h_x[(i, 6)] = b[0];
            h_x[(i, 7)] = b[1];
            h_x[(i, 8)] = b[2];
            h_x[(i, 9)] = c[0];
            h_x[(i, 10)] = c[1];
            h_x[(i, 11)] = c[2];
        }
        h[i] = -norm_p.intensity as f64;
    }
    share.h_x = h_x;
    share.h = h;
}

/// Helper to build the gravity S2 from a raw gravity vector (used by tests).
#[allow(dead_code)]
fn make_gravity(v: &V3D) -> S2 {
    S2::from_vec(v, S2_GRAV_LENGTH, S2_GRAV_TYP)
}

/// Constant referenced for parity with the C++ side.
#[allow(dead_code)]
const _G: f64 = G_M_S2;

#[allow(dead_code)]
fn _lidar_sp_len() -> f32 {
    LIDAR_SP_LEN
}

#[allow(dead_code)]
fn _epss() -> f32 {
    EPSS
}

#[allow(dead_code)]
fn _quat_wxyz(q: &UnitQuaternion<f64>) -> [f64; 4] {
    [q.w, q.i, q.j, q.k]
}

#[allow(dead_code)]
fn _vec3(x: f64, y: f64, z: f64) -> V3D {
    Vector3::new(x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voxel_downsample_reduces() {
        let mut pts = Vec::new();
        for i in 0..100 {
            pts.push(PointType::new(
                0.5 + (i % 5) as f32 * 0.1,
                0.5 + (i % 4) as f32 * 0.1,
                0.5 + (i % 3) as f32 * 0.1,
            ));
        }
        let out = voxel_downsample(&pts, 0.5);
        assert!(!out.is_empty());
        assert!(out.len() <= 20, "out.len = {}", out.len());
    }

    #[test]
    fn esti_plane_on_xy_plane() {
        // non-collinear points on the z=1 plane (a 2D grid)
        let pts: Vec<PointType> = vec![
            PointType::new(0.0, 0.0, 1.0),
            PointType::new(1.0, 0.0, 1.0),
            PointType::new(0.0, 1.0, 1.0),
            PointType::new(1.0, 1.0, 1.0),
            PointType::new(0.5, 0.5, 1.0),
        ];
        let mut pca = [0.0f32; 4];
        let ok = esti_plane(&mut pca, &pts, 0.1);
        assert!(ok);
        // normal should be ~[0,0,-1] with d = 1 (equation -z + 1 = 0 => z = 1)
        assert!(pca[2].abs() > 0.99, "pca = {pca:?}");
        assert!((pca[3] - 1.0).abs() < 0.01, "pca = {pca:?}");
    }

    #[test]
    fn config_default_creates_pipeline() {
        let lm = LaserMapping::new(&LioConfig::default());
        assert!(lm.ikdtree.root.is_none());
        assert_eq!(lm.preprocess.n_scans, 16);
        assert_eq!(lm.imu.lidar_type, LidarType::Avia);
    }
}
