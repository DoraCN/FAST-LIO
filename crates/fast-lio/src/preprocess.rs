//! Point-cloud preprocessing and feature extraction. Ported from
//! `fast_lio/src/preprocess.cpp` / `preprocess.h`.

use crate::types::{
    Ejump, Feature, LidarType, OrgType, PointCloud, PointType, StandardMsg, TimeUnit, AviaMsg,
    SURROUND_NEXT, SURROUND_PREV,
};

/// LiDAR preprocessor.
#[derive(Clone)]
pub struct Preprocess {
    pub feature_enabled: bool,
    pub lidar_type: LidarType,
    pub blind: f64,
    pub point_filter_num: i32,
    pub n_scans: usize,
    pub scan_rate: i32,
    pub time_unit: TimeUnit,
    pub given_offset_time: bool,

    // feature extraction parameters (constructor defaults)
    inf_bound: f64,
    group_size: usize,
    dis_a: f64,
    dis_b: f64,
    p2l_ratio: f64,
    limit_maxmid: f64,
    limit_midmin: f64,
    limit_maxmin: f64,
    jump_up_limit: f64,
    jump_down_limit: f64,
    cos160: f64,
    edgea: f64,
    edgeb: f64,
    smallp_intersect: f64,
    smallp_ratio: f64,

    // working buffers
    pl_full: Vec<PointType>,
    pl_corn: Vec<PointType>,
    pl_surf: Vec<PointType>,
    pl_buff: Vec<Vec<PointType>>,
    typess: Vec<Vec<OrgType>>,
}

impl Preprocess {
    pub fn new() -> Self {
        let mut s = Self {
            feature_enabled: false,
            lidar_type: LidarType::Avia,
            blind: 0.01,
            point_filter_num: 1,
            n_scans: 6,
            scan_rate: 10,
            time_unit: TimeUnit::Us,
            given_offset_time: false,
            inf_bound: 10.0,
            group_size: 8,
            dis_a: 0.1,
            dis_b: 0.0,
            p2l_ratio: 225.0,
            limit_maxmid: 6.25,
            limit_midmin: 6.25,
            limit_maxmin: 3.24,
            jump_up_limit: 170.0,
            jump_down_limit: 8.0,
            cos160: 160.0,
            edgea: 2.0,
            edgeb: 0.1,
            smallp_intersect: 172.5,
            smallp_ratio: 1.2,
            pl_full: Vec::new(),
            pl_corn: Vec::new(),
            pl_surf: Vec::new(),
            pl_buff: Vec::new(),
            typess: Vec::new(),
        };
        s.jump_up_limit = (s.jump_up_limit / 180.0 * std::f64::consts::PI).cos();
        s.jump_down_limit = (s.jump_down_limit / 180.0 * std::f64::consts::PI).cos();
        s.cos160 = (s.cos160 / 180.0 * std::f64::consts::PI).cos();
        s.smallp_intersect = (s.smallp_intersect / 180.0 * std::f64::consts::PI).cos();
        s.pl_buff.resize(s.n_scans, Vec::new());
        s.typess.resize(s.n_scans, Vec::new());
        s
    }

    pub fn set(&mut self, feat_en: bool, lid_type: LidarType, bld: f64, pfilt_num: i32) {
        self.feature_enabled = feat_en;
        self.lidar_type = lid_type;
        self.blind = bld;
        self.point_filter_num = pfilt_num;
    }

    /// Process a Livox Avia message; returns the surf point cloud.
    pub fn process_avia(&mut self, msg: &AviaMsg) -> PointCloud {
        self.avia_handler(msg);
        self.pl_surf.clone()
    }

    /// Process a standard (velodyne/ouster/marsim) message; returns the surf cloud.
    pub fn process_standard(&mut self, msg: &StandardMsg) -> PointCloud {
        match self.lidar_type {
            LidarType::Oust64 => self.oust64_handler(msg),
            LidarType::Velo16 => self.velodyne_handler(msg),
            LidarType::Marsim => self.sim_handler(msg),
            _ => {
                eprintln!("Error LiDAR Type");
            }
        }
        self.pl_surf.clone()
    }

    #[allow(clippy::field_reassign_with_default)]
    fn avia_handler(&mut self, msg: &AviaMsg) {
        self.pl_surf.clear();
        self.pl_corn.clear();
        self.pl_full.clear();
        let plsize = msg.points.len();
        self.pl_corn.reserve(plsize);
        self.pl_surf.reserve(plsize);
        self.pl_full.resize(plsize, PointType::default());
        for i in 0..self.n_scans {
            self.pl_buff[i].clear();
            self.pl_buff[i].reserve(plsize);
        }

        if self.feature_enabled {
            for i in 1..plsize {
                let p = &msg.points[i];
                let line_ok = (p.line as usize) < self.n_scans;
                let tag = (p.tag & 0x30) == 0x10 || (p.tag & 0x30) == 0x00;
                if line_ok && tag {
                    self.pl_full[i].x = p.x;
                    self.pl_full[i].y = p.y;
                    self.pl_full[i].z = p.z;
                    self.pl_full[i].intensity = p.reflectivity as f32;
                    self.pl_full[i].curvature = p.offset_time as f32 / 1_000_000.0;

                    let is_new = (self.pl_full[i].x - self.pl_full[i - 1].x).abs() > 1e-7
                        || (self.pl_full[i].y - self.pl_full[i - 1].y).abs() > 1e-7
                        || (self.pl_full[i].z - self.pl_full[i - 1].z).abs() > 1e-7;
                    if is_new {
                        self.pl_buff[p.line as usize].push(self.pl_full[i]);
                    }
                }
            }
            for j in 0..self.n_scans {
                if self.pl_buff[j].len() <= 5 {
                    continue;
                }
                let pl = self.pl_buff[j].clone();
                let mut plsize = pl.len();
                let mut types = self.typess[j].clone();
                types.clear();
                types.resize(plsize, OrgType::default());
                plsize -= 1;
                for i in 0..plsize {
                    types[i].range = ((pl[i].x * pl[i].x + pl[i].y * pl[i].y) as f64).sqrt();
                    let vx = pl[i].x - pl[i + 1].x;
                    let vy = pl[i].y - pl[i + 1].y;
                    let vz = pl[i].z - pl[i + 1].z;
                    types[i].dista = ((vx * vx + vy * vy + vz * vz) as f64).sqrt();
                }
                types[plsize].range =
                    ((pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y) as f64).sqrt();
                self.give_feature(&pl, &mut types);
            }
        } else {
            let mut valid_num = 0u32;
            for i in 1..plsize {
                let p = &msg.points[i];
                let line_ok = (p.line as usize) < self.n_scans;
                let tag = (p.tag & 0x30) == 0x10 || (p.tag & 0x30) == 0x00;
                if line_ok && tag {
                    valid_num += 1;
                    if valid_num.is_multiple_of(self.point_filter_num as u32) {
                        self.pl_full[i].x = p.x;
                        self.pl_full[i].y = p.y;
                        self.pl_full[i].z = p.z;
                        self.pl_full[i].intensity = p.reflectivity as f32;
                        self.pl_full[i].curvature = p.offset_time as f32 / 1_000_000.0;
                        let is_new = (self.pl_full[i].x - self.pl_full[i - 1].x).abs() > 1e-7
                            || (self.pl_full[i].y - self.pl_full[i - 1].y).abs() > 1e-7
                            || (self.pl_full[i].z - self.pl_full[i - 1].z).abs() > 1e-7;
                        if is_new {
                            let r2 = self.pl_full[i].x * self.pl_full[i].x
                                + self.pl_full[i].y * self.pl_full[i].y
                                + self.pl_full[i].z * self.pl_full[i].z;
                            if (r2 as f64) > (self.blind * self.blind) {
                                self.pl_surf.push(self.pl_full[i]);
                            }
                        }
                    }
                }
            }
        }
    }

    #[allow(clippy::field_reassign_with_default)]
    fn oust64_handler(&mut self, msg: &StandardMsg) {
        self.pl_surf.clear();
        self.pl_corn.clear();
        self.pl_full.clear();
        let plsize = msg.points.len();
        self.pl_corn.reserve(plsize);
        self.pl_surf.reserve(plsize);
        let time_unit_scale = self.time_unit.to_ms_scale();

        if self.feature_enabled {
            for i in 0..self.n_scans {
                self.pl_buff[i].clear();
                self.pl_buff[i].reserve(plsize);
            }
            for i in 0..plsize {
                let p = &msg.points[i];
                let range = p.x * p.x + p.y * p.y + p.z * p.z;
                if (range as f64) < self.blind * self.blind {
                    continue;
                }
                let mut added = PointType::default();
                added.x = p.x;
                added.y = p.y;
                added.z = p.z;
                added.intensity = p.intensity;
                added.curvature = p.time * time_unit_scale;
                if (p.ring as usize) < self.n_scans {
                    self.pl_buff[p.ring as usize].push(added);
                }
            }
            for j in 0..self.n_scans {
                let pl = self.pl_buff[j].clone();
                let mut linesize = pl.len();
                let mut types = self.typess[j].clone();
                types.clear();
                types.resize(linesize, OrgType::default());
                if linesize == 0 {
                    continue;
                }
                linesize -= 1;
                for i in 0..linesize {
                    types[i].range = ((pl[i].x * pl[i].x + pl[i].y * pl[i].y) as f64).sqrt();
                    let vx = pl[i].x - pl[i + 1].x;
                    let vy = pl[i].y - pl[i + 1].y;
                    let vz = pl[i].z - pl[i + 1].z;
                    types[i].dista = (vx * vx + vy * vy + vz * vz) as f64;
                }
                types[linesize].range =
                    ((pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y) as f64)
                        .sqrt();
                self.give_feature(&pl, &mut types);
            }
        } else {
            for (i, p) in msg.points.iter().enumerate() {
                if i % self.point_filter_num as usize != 0 {
                    continue;
                }
                let range = p.x * p.x + p.y * p.y + p.z * p.z;
                if (range as f64) < self.blind * self.blind {
                    continue;
                }
                let mut added = PointType::default();
                added.x = p.x;
                added.y = p.y;
                added.z = p.z;
                added.intensity = p.intensity;
                added.curvature = p.time * time_unit_scale;
                self.pl_surf.push(added);
            }
        }
    }

    #[allow(clippy::field_reassign_with_default)]
    fn velodyne_handler(&mut self, msg: &StandardMsg) {
        self.pl_surf.clear();
        self.pl_corn.clear();
        self.pl_full.clear();
        let plsize = msg.points.len();
        if plsize == 0 {
            return;
        }
        self.pl_surf.reserve(plsize);
        let time_unit_scale = self.time_unit.to_ms_scale();

        let omega_l = 0.361 * self.scan_rate as f64;
        let mut is_first = vec![true; self.n_scans];
        let mut yaw_fp = vec![0.0f64; self.n_scans];
        let mut yaw_last = vec![0.0f32; self.n_scans];
        let mut time_last = vec![0.0f32; self.n_scans];

        self.given_offset_time = msg.points[plsize - 1].time > 0.0;

        if self.feature_enabled {
            for i in 0..self.n_scans {
                self.pl_buff[i].clear();
                self.pl_buff[i].reserve(plsize);
            }
            for i in 0..plsize {
                let p = &msg.points[i];
                let layer = p.ring as usize;
                if layer >= self.n_scans {
                    continue;
                }
                let mut added = PointType::default();
                added.x = p.x;
                added.y = p.y;
                added.z = p.z;
                added.intensity = p.intensity;
                added.curvature = p.time * time_unit_scale;

                if !self.given_offset_time {
                    let yaw_angle = (p.y as f64).atan2(p.x as f64) * 57.2957;
                    if is_first[layer] {
                        yaw_fp[layer] = yaw_angle;
                        is_first[layer] = false;
                        added.curvature = 0.0;
                        yaw_last[layer] = yaw_angle as f32;
                        time_last[layer] = added.curvature;
                        self.pl_buff[layer].push(added);
                        continue;
                    }
                    added.curvature = if yaw_angle <= yaw_fp[layer] {
                        ((yaw_fp[layer] - yaw_angle) / omega_l) as f32
                    } else {
                        ((yaw_fp[layer] - yaw_angle + 360.0) / omega_l) as f32
                    };
                    if added.curvature < time_last[layer] {
                        added.curvature += (360.0 / omega_l) as f32;
                    }
                    yaw_last[layer] = yaw_angle as f32;
                    time_last[layer] = added.curvature;
                }
                self.pl_buff[layer].push(added);
            }
            for j in 0..self.n_scans {
                let pl = self.pl_buff[j].clone();
                let mut linesize = pl.len();
                if linesize < 2 {
                    continue;
                }
                let mut types = self.typess[j].clone();
                types.clear();
                types.resize(linesize, OrgType::default());
                linesize -= 1;
                for i in 0..linesize {
                    types[i].range = ((pl[i].x * pl[i].x + pl[i].y * pl[i].y) as f64).sqrt();
                    let vx = pl[i].x - pl[i + 1].x;
                    let vy = pl[i].y - pl[i + 1].y;
                    let vz = pl[i].z - pl[i + 1].z;
                    types[i].dista = (vx * vx + vy * vy + vz * vz) as f64;
                }
                types[linesize].range =
                    ((pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y) as f64)
                        .sqrt();
                self.give_feature(&pl, &mut types);
            }
        } else {
            for i in 0..plsize {
                let p = &msg.points[i];
                let mut added = PointType::default();
                added.x = p.x;
                added.y = p.y;
                added.z = p.z;
                added.intensity = p.intensity;
                added.curvature = p.time * time_unit_scale;

                if !self.given_offset_time {
                    let layer = p.ring as usize;
                    let yaw_angle = (p.y as f64).atan2(p.x as f64) * 57.2957;
                    if is_first[layer] {
                        yaw_fp[layer] = yaw_angle;
                        is_first[layer] = false;
                        added.curvature = 0.0;
                        yaw_last[layer] = yaw_angle as f32;
                        time_last[layer] = added.curvature;
                        if i % self.point_filter_num as usize == 0 {
                            let r2 = added.x * added.x + added.y * added.y + added.z * added.z;
                            if (r2 as f64) > (self.blind * self.blind) {
                                self.pl_surf.push(added);
                            }
                        }
                        continue;
                    }
                    added.curvature = if yaw_angle <= yaw_fp[layer] {
                        ((yaw_fp[layer] - yaw_angle) / omega_l) as f32
                    } else {
                        ((yaw_fp[layer] - yaw_angle + 360.0) / omega_l) as f32
                    };
                    if added.curvature < time_last[layer] {
                        added.curvature += (360.0 / omega_l) as f32;
                    }
                    yaw_last[layer] = yaw_angle as f32;
                    time_last[layer] = added.curvature;
                }
                if i % self.point_filter_num as usize == 0 {
                    let r2 = added.x * added.x + added.y * added.y + added.z * added.z;
                    if (r2 as f64) > (self.blind * self.blind) {
                        self.pl_surf.push(added);
                    }
                }
            }
        }
    }

    #[allow(clippy::field_reassign_with_default)]
    fn sim_handler(&mut self, msg: &StandardMsg) {
        self.pl_surf.clear();
        self.pl_full.clear();
        let plsize = msg.points.len();
        self.pl_surf.reserve(plsize);
        for p in &msg.points {
            let range = p.x * p.x + p.y * p.y + p.z * p.z;
            if (range as f64) < self.blind * self.blind {
                continue;
            }
            let mut added = PointType::default();
            added.x = p.x;
            added.y = p.y;
            added.z = p.z;
            added.intensity = p.intensity;
            self.pl_surf.push(added);
        }
    }

    /// `give_feature`: classify points into planes / edges and fill `pl_surf`.
    #[allow(clippy::needless_range_loop)]
    fn give_feature(&mut self, pl: &[PointType], types: &mut [OrgType]) {
        let plsize = pl.len();
        if plsize == 0 {
            return;
        }
        let mut head = 0usize;
        while head < plsize && types[head].range < self.blind {
            head += 1;
        }

        // Surf detection
        let plsize2 = plsize.saturating_sub(self.group_size);
        let mut curr_direct = [0.0f64; 3];
        let mut last_direct = [0.0f64; 3];
        let mut i_nex = 0usize;
        let mut last_state = 0;

        let mut i = head;
        while i < plsize2 {
            if types[i].range < self.blind {
                i += 1;
                continue;
            }
            let plane_type = self.plane_judge(pl, types, i, &mut i_nex, &mut curr_direct);
            if plane_type == 1 {
                for j in i..=i_nex {
                    types[j].ftype = if j != i && j != i_nex {
                        Feature::RealPlane
                    } else {
                        Feature::PossPlane
                    };
                }
                if last_state == 1 && last_direct[0] * last_direct[0] + last_direct[1] * last_direct[1] + last_direct[2] * last_direct[2] > 0.01 {
                    let mod_ = last_direct[0] * curr_direct[0]
                        + last_direct[1] * curr_direct[1]
                        + last_direct[2] * curr_direct[2];
                    types[i].ftype = if mod_ > -0.707 && mod_ < 0.707 {
                        Feature::EdgePlane
                    } else {
                        Feature::RealPlane
                    };
                }
                i = i_nex.saturating_sub(1);
                last_state = 1;
            } else {
                i = i_nex;
                last_state = 0;
            }
            last_direct = curr_direct;
            i += 1;
        }

        // Edge detection
        let plsize2 = plsize.saturating_sub(3);
        let mut i = head + 3;
        while i < plsize2 {
            if types[i].range < self.blind || types[i].ftype >= Feature::RealPlane {
                i += 1;
                continue;
            }
            if types[i - 1].dista < 1e-16 || types[i].dista < 1e-16 {
                i += 1;
                continue;
            }
            let vec_a = [pl[i].x as f64, pl[i].y as f64, pl[i].z as f64];
            let mut vecs = [[0.0f64; 3]; 2];
            for j in 0..2 {
                let m: i32 = if j == 0 { -1 } else { 1 };
                let idx = i as i64 + m as i64;
                if idx < 0 || idx >= plsize as i64 {
                    i += 1;
                    continue;
                }
                let idx = idx as usize;
                if types[idx].range < self.blind {
                    types[i].edj[j] = if types[i].range > self.inf_bound { Ejump::NrInf } else { Ejump::NrBlind };
                    continue;
                }
                vecs[j] = [
                    pl[idx].x as f64 - vec_a[0],
                    pl[idx].y as f64 - vec_a[1],
                    pl[idx].z as f64 - vec_a[2],
                ];
                let na = (vec_a[0] * vec_a[0] + vec_a[1] * vec_a[1] + vec_a[2] * vec_a[2]).sqrt();
                let nv = (vecs[j][0] * vecs[j][0] + vecs[j][1] * vecs[j][1] + vecs[j][2] * vecs[j][2])
                    .sqrt();
                types[i].angle[j] =
                    (vec_a[0] * vecs[j][0] + vec_a[1] * vecs[j][1] + vec_a[2] * vecs[j][2]) / na / nv;
                if types[i].angle[j] < self.jump_up_limit {
                    types[i].edj[j] = Ejump::Nr180;
                } else if types[i].angle[j] > self.jump_down_limit {
                    types[i].edj[j] = Ejump::NrZero;
                }
            }
            types[i].intersect = (vecs[0][0] * vecs[1][0]
                + vecs[0][1] * vecs[1][1]
                + vecs[0][2] * vecs[1][2])
                / ((vecs[0][0] * vecs[0][0] + vecs[0][1] * vecs[0][1] + vecs[0][2] * vecs[0][2]).sqrt()
                    * (vecs[1][0] * vecs[1][0] + vecs[1][1] * vecs[1][1] + vecs[1][2] * vecs[1][2])
                        .sqrt());
            if types[i].edj[SURROUND_PREV] == Ejump::NrNor
                && types[i].edj[SURROUND_NEXT] == Ejump::NrZero
                && types[i].dista > 0.0225
                && types[i].dista > 4.0 * types[i - 1].dista
            {
                if types[i].intersect > self.cos160 && self.edge_jump_judge(pl, types, i, SURROUND_PREV) {
                    types[i].ftype = Feature::EdgeJump;
                }
            } else if types[i].edj[SURROUND_PREV] == Ejump::NrZero
                && types[i].edj[SURROUND_NEXT] == Ejump::NrNor
                && types[i - 1].dista > 0.0225
                && types[i - 1].dista > 4.0 * types[i].dista
            {
                if types[i].intersect > self.cos160 && self.edge_jump_judge(pl, types, i, SURROUND_NEXT) {
                    types[i].ftype = Feature::EdgeJump;
                }
            } else if types[i].edj[SURROUND_PREV] == Ejump::NrNor && types[i].edj[SURROUND_NEXT] == Ejump::NrInf {
                if self.edge_jump_judge(pl, types, i, SURROUND_PREV) {
                    types[i].ftype = Feature::EdgeJump;
                }
            } else if types[i].edj[SURROUND_PREV] == Ejump::NrInf && types[i].edj[SURROUND_NEXT] == Ejump::NrNor {
                if self.edge_jump_judge(pl, types, i, SURROUND_NEXT) {
                    types[i].ftype = Feature::EdgeJump;
                }
            } else if types[i].edj[SURROUND_PREV] > Ejump::NrNor
                && types[i].edj[SURROUND_NEXT] > Ejump::NrNor
                && types[i].ftype == Feature::Nor {
                    types[i].ftype = Feature::Wire;
                }
            i += 1;
        }

        // Small-plane merge
        let plsize2 = plsize - 1;
        let mut i = head + 1;
        while i < plsize2 {
            if types[i].range < self.blind || types[i - 1].range < self.blind || types[i + 1].range < self.blind {
                i += 1;
                continue;
            }
            if types[i - 1].dista < 1e-8 || types[i].dista < 1e-8 {
                i += 1;
                continue;
            }
            if types[i].ftype == Feature::Nor {
                let ratio = if types[i - 1].dista > types[i].dista {
                    types[i - 1].dista / types[i].dista
                } else {
                    types[i].dista / types[i - 1].dista
                };
                if types[i].intersect < self.smallp_intersect && ratio < self.smallp_ratio {
                    if types[i - 1].ftype == Feature::Nor {
                        types[i - 1].ftype = Feature::RealPlane;
                    }
                    if types[i + 1].ftype == Feature::Nor {
                        types[i + 1].ftype = Feature::RealPlane;
                    }
                    types[i].ftype = Feature::RealPlane;
                }
            }
            i += 1;
        }

        // Surface / corner output
        let mut last_surface: i64 = -1;
        let mut j = head;
        while j < plsize {
            if types[j].ftype == Feature::PossPlane || types[j].ftype == Feature::RealPlane {
                if last_surface == -1 {
                    last_surface = j as i64;
                }
                if j as i64 == last_surface + self.point_filter_num as i64 - 1 {
                    let ap = PointType {
                        x: pl[j].x,
                        y: pl[j].y,
                        z: pl[j].z,
                        intensity: pl[j].intensity,
                        curvature: pl[j].curvature,
                        ..Default::default()
                    };
                    self.pl_surf.push(ap);
                    last_surface = -1;
                }
            } else {
                if types[j].ftype == Feature::EdgeJump || types[j].ftype == Feature::EdgePlane {
                    self.pl_corn.push(pl[j]);
                }
                if last_surface != -1 {
                    let mut ap = PointType::default();
                    let start = last_surface as usize;
                    for pk in &pl[start..j] {
                        ap.x += pk.x;
                        ap.y += pk.y;
                        ap.z += pk.z;
                        ap.intensity += pk.intensity;
                        ap.curvature += pk.curvature;
                    }
                    let n = (j - start) as f32;
                    ap.x /= n;
                    ap.y /= n;
                    ap.z /= n;
                    ap.intensity /= n;
                    ap.curvature /= n;
                    self.pl_surf.push(ap);
                }
                last_surface = -1;
            }
            j += 1;
        }
    }

    /// `plane_judge`: returns 1 if a plane segment is found, otherwise 0/2.
    fn plane_judge(
        &self,
        pl: &[PointType],
        types: &[OrgType],
        i_cur: usize,
        i_nex: &mut usize,
        curr_direct: &mut [f64; 3],
    ) -> i32 {
        let mut group_dis = self.dis_a * types[i_cur].range + self.dis_b;
        group_dis = group_dis * group_dis;
        let mut two_dis = 0.0f64;
        let mut disarr: Vec<f64> = Vec::with_capacity(20);

        *i_nex = i_cur;
        while *i_nex < i_cur + self.group_size {
            if types[*i_nex].range < self.blind {
                *curr_direct = [0.0, 0.0, 0.0];
                return 2;
            }
            disarr.push(types[*i_nex].dista);
            *i_nex += 1;
        }

        loop {
            if i_cur >= pl.len() || *i_nex >= pl.len() {
                break;
            }
            if types[*i_nex].range < self.blind {
                *curr_direct = [0.0, 0.0, 0.0];
                return 2;
            }
            let vx = pl[*i_nex].x - pl[i_cur].x;
            let vy = pl[*i_nex].y - pl[i_cur].y;
            let vz = pl[*i_nex].z - pl[i_cur].z;
            two_dis = (vx * vx + vy * vy + vz * vz) as f64;
            if two_dis >= group_dis {
                break;
            }
            disarr.push(types[*i_nex].dista);
            *i_nex += 1;
        }

        let mut leng_wid = 0.0f64;
        let mut j = i_cur + 1;
        while j < *i_nex {
            if j >= pl.len() || i_cur >= pl.len() {
                break;
            }
            let v1 = [
                pl[j].x - pl[i_cur].x,
                pl[j].y - pl[i_cur].y,
                pl[j].z - pl[i_cur].z,
            ];
            let v2x = pl[*i_nex].x - pl[i_cur].x;
            let v2y = pl[*i_nex].y - pl[i_cur].y;
            let v2z = pl[*i_nex].z - pl[i_cur].z;
            let vx = v2x as f64;
            let vy = v2y as f64;
            let vz = v2z as f64;
            let c0 = v1[1] as f64 * vz - vy * v1[2] as f64;
            let c1 = v1[2] as f64 * vx - v1[0] as f64 * vz;
            let c2 = v1[0] as f64 * vy - vx * v1[1] as f64;
            let lw = c0 * c0 + c1 * c1 + c2 * c2;
            if lw > leng_wid {
                leng_wid = lw;
            }
            j += 1;
        }

        if two_dis * two_dis / leng_wid < self.p2l_ratio {
            *curr_direct = [0.0, 0.0, 0.0];
            return 0;
        }

        let disarrsize = disarr.len();
        for j in 0..disarrsize.saturating_sub(1) {
            for k in (j + 1)..disarrsize {
                if disarr[j] < disarr[k] {
                    disarr.swap(j, k);
                }
            }
        }
        if disarr[disarr.len() - 2] < 1e-16 {
            *curr_direct = [0.0, 0.0, 0.0];
            return 0;
        }
        if self.lidar_type == LidarType::Avia {
            let dismax_mid = disarr[0] / disarr[disarrsize / 2];
            let dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];
            if dismax_mid >= self.limit_maxmid || dismid_min >= self.limit_midmin {
                *curr_direct = [0.0, 0.0, 0.0];
                return 0;
            }
        } else {
            let dismax_min = disarr[0] / disarr[disarrsize - 2];
            if dismax_min >= self.limit_maxmin {
                *curr_direct = [0.0, 0.0, 0.0];
                return 0;
            }
        }
        let vx = pl[*i_nex].x - pl[i_cur].x;
        let vy = pl[*i_nex].y - pl[i_cur].y;
        let vz = pl[*i_nex].z - pl[i_cur].z;
        let n = ((vx * vx + vy * vy + vz * vz) as f64).sqrt();
        *curr_direct = [vx as f64 / n, vy as f64 / n, vz as f64 / n];
        1
    }

    /// `edge_jump_judge`: decide whether an edge jump is a valid edge feature.
    fn edge_jump_judge(&self, pl: &[PointType], types: &[OrgType], i: usize, nor_dir: usize) -> bool {
        if nor_dir == SURROUND_PREV {
            if i < 2 {
                return false;
            }
            if types[i - 1].range < self.blind || types[i - 2].range < self.blind {
                return false;
            }
        } else if nor_dir == SURROUND_NEXT {
            if i + 2 >= pl.len() {
                return false;
            }
            if types[i + 1].range < self.blind || types[i + 2].range < self.blind {
                return false;
            }
        }
        // indices: i+nor_dir-1 and i+3*nor_dir-2 with nor_dir in {0,1}
        let (d1, d2) = if nor_dir == 0 {
            (types[i - 1].dista, types[i - 2].dista)
        } else {
            (types[i].dista, types[i + 1].dista)
        };
        let (mut d1, mut d2) = (d1, d2);
        if d1 < d2 {
            std::mem::swap(&mut d1, &mut d2);
        }
        d1 = d1.sqrt();
        d2 = d2.sqrt();
        !(d1 > self.edgea * d2 || (d1 - d2) > self.edgeb)
    }
}


impl Default for Preprocess {
    fn default() -> Self {
        Self::new()
    }
}
