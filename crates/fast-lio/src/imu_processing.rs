//! IMU processing: initialization, forward propagation and lidar-point
//! undistortion. Ported from `fast_lio/src/IMU_Processing.hpp`.

use nalgebra::{SMatrix, UnitQuaternion};

use crate::consts::G_M_S2;
use crate::esekf::{Cov, EseKf, PROC_N};
use crate::math::so3::{exp_scaled, M3D, V3D};
use crate::model::{process_noise_cov, InputIkfom};
use crate::types::{LidarType, MeasureGroup, PointCloud, PointType};

const MAX_INI_COUNT: i32 = 10;

/// Pose of the IMU at one timestamp (used for backward undistortion).
#[derive(Clone, Copy, Debug)]
pub struct Pose6D {
    pub offset_time: f64,
    pub acc: V3D,
    pub gyr: V3D,
    pub vel: V3D,
    pub pos: V3D,
    pub rot: M3D,
}

/// IMU processor.
#[derive(Clone)]
pub struct ImuProcess {
    pub cov_acc: V3D,
    pub cov_gyr: V3D,
    pub cov_acc_scale: V3D,
    pub cov_gyr_scale: V3D,
    pub cov_bias_gyr: V3D,
    pub cov_bias_acc: V3D,
    pub first_lidar_time: f64,
    pub lidar_type: LidarType,
    pub q: SMatrix<f64, PROC_N, PROC_N>,

    mean_acc: V3D,
    mean_gyr: V3D,
    angvel_last: V3D,
    acc_s_last: V3D,
    lidar_r_wrt_imu: M3D,
    lidar_t_wrt_imu: V3D,
    start_timestamp: f64,
    last_lidar_end_time: f64,
    init_iter_num: i32,
    b_first_frame: bool,
    imu_need_init: bool,
    last_imu: Option<crate::types::ImuRaw>,
    imu_pose: Vec<Pose6D>,
}

impl Default for ImuProcess {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuProcess {
    pub fn new() -> Self {
        Self {
            cov_acc: V3D::new(0.1, 0.1, 0.1),
            cov_gyr: V3D::new(0.1, 0.1, 0.1),
            cov_acc_scale: V3D::zeros(),
            cov_gyr_scale: V3D::zeros(),
            cov_bias_gyr: V3D::new(0.0001, 0.0001, 0.0001),
            cov_bias_acc: V3D::new(0.0001, 0.0001, 0.0001),
            first_lidar_time: 0.0,
            lidar_type: LidarType::Avia,
            q: process_noise_cov(),
            mean_acc: V3D::new(0.0, 0.0, -1.0),
            mean_gyr: V3D::zeros(),
            angvel_last: V3D::zeros(),
            acc_s_last: V3D::zeros(),
            lidar_r_wrt_imu: M3D::identity(),
            lidar_t_wrt_imu: V3D::zeros(),
            start_timestamp: -1.0,
            last_lidar_end_time: 0.0,
            init_iter_num: 1,
            b_first_frame: true,
            imu_need_init: true,
            last_imu: None,
            imu_pose: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        self.mean_acc = V3D::new(0.0, 0.0, -1.0);
        self.mean_gyr = V3D::zeros();
        self.angvel_last = V3D::zeros();
        self.imu_need_init = true;
        self.start_timestamp = -1.0;
        self.init_iter_num = 1;
        self.imu_pose.clear();
        self.last_imu = None;
    }

    pub fn set_extrinsic(&mut self, transl: &V3D, rot: &M3D) {
        self.lidar_t_wrt_imu = *transl;
        self.lidar_r_wrt_imu = *rot;
    }

    pub fn set_gyr_cov(&mut self, scaler: &V3D) {
        self.cov_gyr_scale = *scaler;
    }

    pub fn set_acc_cov(&mut self, scaler: &V3D) {
        self.cov_acc_scale = *scaler;
    }

    pub fn set_gyr_bias_cov(&mut self, b_g: &V3D) {
        self.cov_bias_gyr = *b_g;
    }

    pub fn set_acc_bias_cov(&mut self, b_a: &V3D) {
        self.cov_bias_acc = *b_a;
    }

    /// Process a measurement group: first ~10 frames perform IMU init, then
    /// undistortion. Writes the undistorted point cloud into `pcl_un`.
    pub fn process(&mut self, meas: &MeasureGroup, kf_state: &mut EseKf, pcl_un: &mut PointCloud) {
        if meas.imu.is_empty() {
            return;
        }
        if self.imu_need_init {
            self.imu_init(meas, kf_state);
            // dead assignment kept for C++ fidelity
            self.imu_need_init = true;
            self.last_imu = meas.imu.last().copied();
            if self.init_iter_num > MAX_INI_COUNT {
                self.cov_acc *= (G_M_S2 / self.mean_acc.norm()).powi(2);
                self.imu_need_init = false;
                self.cov_acc = self.cov_acc_scale;
                self.cov_gyr = self.cov_gyr_scale;
            }
            return;
        }
        self.undistort_pcl(meas, kf_state, pcl_un);
    }

    /// Accumulate IMU statistics to estimate gravity / biases / covariances.
    fn imu_init(&mut self, meas: &MeasureGroup, kf_state: &mut EseKf) {
        if self.b_first_frame {
            self.reset();
            let n = 1i32;
            self.b_first_frame = false;
            let first = meas.imu[0];
            self.mean_acc = V3D::new(first.acc[0], first.acc[1], first.acc[2]);
            self.mean_gyr = V3D::new(first.gyr[0], first.gyr[1], first.gyr[2]);
            self.first_lidar_time = meas.lidar_beg_time;
            let _ = n;
        }
        let mut n = self.init_iter_num as f64;
        for imu in &meas.imu {
            let cur_acc = V3D::new(imu.acc[0], imu.acc[1], imu.acc[2]);
            let cur_gyr = V3D::new(imu.gyr[0], imu.gyr[1], imu.gyr[2]);
            self.mean_acc += (cur_acc - self.mean_acc) / n;
            self.mean_gyr += (cur_gyr - self.mean_gyr) / n;
            let da = cur_acc - self.mean_acc;
            let dg = cur_gyr - self.mean_gyr;
            self.cov_acc = self.cov_acc * (n - 1.0) / n
                + V3D::new(da[0] * da[0], da[1] * da[1], da[2] * da[2]) * (n - 1.0) / (n * n);
            self.cov_gyr = self.cov_gyr * (n - 1.0) / n
                + V3D::new(dg[0] * dg[0], dg[1] * dg[1], dg[2] * dg[2]) * (n - 1.0) / (n * n);
            n += 1.0;
        }
        self.init_iter_num = n as i32;

        let mut init_state = kf_state.get_x().clone();
        init_state.grav = crate::math::s2::S2::from_vec(
            &(-self.mean_acc / self.mean_acc.norm() * G_M_S2),
            crate::math::manifold::S2_GRAV_LENGTH,
            crate::math::manifold::S2_GRAV_TYP,
        );
        init_state.bg = self.mean_gyr;
        init_state.offset_t_l_i = self.lidar_t_wrt_imu;
        init_state.offset_r_l_i = UnitQuaternion::from_rotation_matrix(
            &nalgebra::Rotation3::from_matrix_unchecked(self.lidar_r_wrt_imu),
        );
        kf_state.change_x(init_state);

        let mut init_p: Cov = Cov::identity();
        for i in 0..3 {
            init_p[(6 + i, 6 + i)] = 0.00001;
            init_p[(9 + i, 9 + i)] = 0.00001;
            init_p[(15 + i, 15 + i)] = 0.0001;
            init_p[(18 + i, 18 + i)] = 0.001;
        }
        init_p[(21, 21)] = 0.00001;
        init_p[(22, 22)] = 0.00001;
        kf_state.change_p(init_p);

        self.last_imu = meas.imu.last().copied();
    }

    /// Forward-propagate the state over the IMU measurements and undistort the
    /// lidar points back to the frame-end pose.
    fn undistort_pcl(&mut self, meas: &MeasureGroup, kf_state: &mut EseKf, pcl_out: &mut PointCloud) {
        let mut v_imu: Vec<crate::types::ImuRaw> = meas.imu.clone();
        if let Some(last) = self.last_imu {
            v_imu.insert(0, last);
        }
        if v_imu.is_empty() {
            return;
        }
        let imu_end_time = v_imu[v_imu.len() - 1].stamp;

        let mut pcl_beg_time = meas.lidar_beg_time;
        let pcl_end_time = meas.lidar_end_time;
        if self.lidar_type == LidarType::Marsim {
            pcl_beg_time = self.last_lidar_end_time;
        }

        // sort points by offset time (curvature is in ms)
        *pcl_out = meas.lidar.clone();
        pcl_out.sort_by(|a, b| a.curvature.partial_cmp(&b.curvature).unwrap());

        let mut imu_state = kf_state.get_x().clone();
        self.imu_pose.clear();
        self.imu_pose.push(Pose6D {
            offset_time: 0.0,
            acc: self.acc_s_last,
            gyr: self.angvel_last,
            vel: imu_state.vel,
            pos: imu_state.pos,
            rot: imu_state.rot.to_rotation_matrix().into(),
        });

        let mut input = InputIkfom::default();
        #[allow(unused_assignments)]
        let mut dt = 0.0f64;
        for k in 0..v_imu.len().saturating_sub(1) {
            let head = v_imu[k];
            let tail = v_imu[k + 1];
            if tail.stamp < self.last_lidar_end_time {
                continue;
            }
            let angvel_avr = 0.5 * (V3D::from_column_slice(&head.gyr) + V3D::from_column_slice(&tail.gyr));
            let mut acc_avr = 0.5 * (V3D::from_column_slice(&head.acc) + V3D::from_column_slice(&tail.acc));
            acc_avr = acc_avr * G_M_S2 / self.mean_acc.norm();

            dt = if head.stamp < self.last_lidar_end_time {
                tail.stamp - self.last_lidar_end_time
            } else {
                tail.stamp - head.stamp
            };

            input.acc = acc_avr;
            input.gyro = angvel_avr;
            for i in 0..3 {
                self.q[(i, i)] = self.cov_gyr[i];
                self.q[(3 + i, 3 + i)] = self.cov_acc[i];
                self.q[(6 + i, 6 + i)] = self.cov_bias_gyr[i];
                self.q[(9 + i, 9 + i)] = self.cov_bias_acc[i];
            }
            kf_state.predict(dt, &self.q, &input);

            imu_state = kf_state.get_x().clone();
            self.angvel_last = angvel_avr - imu_state.bg;
            self.acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
            self.acc_s_last += imu_state.grav.vec;
            let offs_t = tail.stamp - pcl_beg_time;
            self.imu_pose.push(Pose6D {
                offset_time: offs_t,
                acc: self.acc_s_last,
                gyr: self.angvel_last,
                vel: imu_state.vel,
                pos: imu_state.pos,
                rot: imu_state.rot.to_rotation_matrix().into(),
            });
        }

        // propagate to the frame end
        let note = if pcl_end_time > imu_end_time { 1.0 } else { -1.0 };
        dt = note * (pcl_end_time - imu_end_time);
        kf_state.predict(dt, &self.q, &input);
        imu_state = kf_state.get_x().clone();
        self.last_imu = meas.imu.last().copied();
        self.last_lidar_end_time = pcl_end_time;

        if pcl_out.is_empty() || self.lidar_type == LidarType::Marsim {
            return;
        }

        // backward undistortion
        let mut it_pcl = pcl_out.len() - 1;
        for kp in (1..self.imu_pose.len()).rev() {
            let head = self.imu_pose[kp - 1];
            let tail = self.imu_pose[kp];
            let r_imu = head.rot;
            let vel_imu = head.vel;
            let pos_imu = head.pos;
            let acc_imu = tail.acc;
            let angvel_avr = tail.gyr;

            while (pcl_out[it_pcl].curvature as f64 / 1000.0) > head.offset_time {
                let dt = pcl_out[it_pcl].curvature as f64 / 1000.0 - head.offset_time;
                let r_i = r_imu * exp_scaled(&angvel_avr, dt);
                let p_i = V3D::new(pcl_out[it_pcl].x as f64, pcl_out[it_pcl].y as f64, pcl_out[it_pcl].z as f64);
                let t_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos;
                let inner = r_i * (imu_state.offset_r_l_i * p_i + imu_state.offset_t_l_i) + t_ei;
                let p_compensate = imu_state.offset_r_l_i.conjugate() * (imu_state.rot.conjugate() * inner - imu_state.offset_t_l_i);
                pcl_out[it_pcl].x = p_compensate[0] as f32;
                pcl_out[it_pcl].y = p_compensate[1] as f32;
                pcl_out[it_pcl].z = p_compensate[2] as f32;
                if it_pcl == 0 {
                    break;
                }
                it_pcl -= 1;
            }
        }
    }
}

/// Comparator helper: keep a reference to the `time_list` used for sorting.
#[allow(dead_code)]
fn time_less(a: &PointType, b: &PointType) -> bool {
    a.curvature < b.curvature
}
