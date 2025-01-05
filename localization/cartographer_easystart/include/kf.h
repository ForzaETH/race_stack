#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "utils.h"

// constexpr int kStateDim = 15;
// constexpr int kNoiseDim = 12;

//TODORYU
// Ulsan UNIST 3dmgq7
// constexpr double kG = 9.78662;
// Daegu PG 3dmgq7
constexpr double kG = 9.80186;

// using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

class KF {
   public:
    // struct State {
    //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //     // error-state
    //     MatrixSD cov;

    //     // nominal-state
    //     Eigen::Vector3d p_GI;
    //     Eigen::Vector3d v_GI;
    //     Eigen::Vector3d av_GI;
    //     Eigen::Matrix3d r_GI;
    //     Eigen::Vector3d acc_bias;
    //     Eigen::Vector3d gyr_bias;

    //     double timestamp;

    //     State() {
    //         cov.setZero();

    //         p_GI.setZero();
    //         v_GI.setZero();
    //         r_GI.setIdentity();

    //         acc_bias.setZero();
    //         gyr_bias.setZero();
    //     }
    // };
    // using StatePtr = std::shared_ptr<State>;
    StatePtr state_ptr_;

    KF() = delete;

    KF(const KF &) = delete;

    explicit KF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8, double sig_pv = 10., double sig_rp_deg = 10., double sig_yaw_deg = 100.)
        : acc_noise_(acc_n),
          gyr_noise_(gyr_n),
          acc_bias_noise_(acc_w),
          gyr_bias_noise_(gyr_w),
          sigma_posvel_(sig_pv),
          sigma_rolpit_deg_(sig_rp_deg),
          sigma_yaw_deg_(sig_yaw_deg) {
        state_ptr_ = std::make_shared<State>();

        const double kDegreeToRadian = M_PI / 180.;

        const double sigma_rp = sig_rp_deg * kDegreeToRadian;
        const double sigma_yaw = sigma_yaw_deg_ * kDegreeToRadian;

        state_ptr_->cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_posvel_*sigma_posvel_;                 // position std: 10 m
        state_ptr_->cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 100.;                 // velocity std: 10 m/s
        state_ptr_->cov.block<2, 2>(6, 6) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;  // roll pitch std 10 degree
        state_ptr_->cov(8, 8) = sigma_yaw * sigma_yaw;                                          // yaw std: 100 degree
        state_ptr_->cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.0004;               // Acc bias
        state_ptr_->cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.0004;             // Gyr bias
    }

    /**
     * @brief predict procedure
     * @ref ESKF 5.4: System kinematics in discrete time
     * @param last_imu 
     * @param curr_imu 
     */
    void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
        const double dt = curr_imu->timestamp - last_imu->timestamp;
        const double dt2 = dt * dt;

        State last_state = *state_ptr_;

        // timestamp
        state_ptr_->timestamp = curr_imu->timestamp;

        //
        // ESKF 5.4.1 The nominal state kinematics
        //

        // p v R
        const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias;
        const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias;

        // std::cout <<acc_unbias<< std::endl;
        const Eigen::Vector3d acc_nominal = last_state.r_GI * acc_unbias + Eigen::Vector3d(0, 0, -kG);    
        // std::cout <<"!!!!!!!!!!!!!!!!!!!"<< std::endl;
        // std::cout <<last_state.r_GI * acc_unbias    << std::endl;
        // std::cout << last_state.v_GI << std::endl;

        state_ptr_->p_GI = last_state.p_GI + last_state.v_GI * dt + 0.5 * acc_nominal * dt2;
        state_ptr_->v_GI = last_state.v_GI + acc_nominal * dt;
        // std::cout <<state_ptr_->v_GI[2]<< std::endl;
        const Eigen::Vector3d delta_angle_axis = gyr_unbias * dt;
        double norm_delta_angle = delta_angle_axis.norm();
        Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
        if (norm_delta_angle > DBL_EPSILON) {
            dR = Eigen::AngleAxisd(norm_delta_angle, delta_angle_axis.normalized()).toRotationMatrix();
            state_ptr_->r_GI = last_state.r_GI * dR;
        }
        state_ptr_->av_GI = gyr_unbias; 
        //
        // ESKF 5.4.3 The error-state Jacobian and perturbation matrices
        //

        // Fx
        MatrixSD Fx = MatrixSD::Identity();
        Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        Fx.block<3, 3>(3, 6) = -state_ptr_->r_GI * skew_matrix(acc_unbias) * dt;
        Fx.block<3, 3>(3, 9) = -state_ptr_->r_GI * dt;
        if (norm_delta_angle > DBL_EPSILON) {
            Fx.block<3, 3>(6, 6) = dR.transpose();
        } else {
            Fx.block<3, 3>(6, 6).setIdentity();
        }
        Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        // Fi
        Eigen::Matrix<double, kStateDim, kNoiseDim> Fi = Eigen::Matrix<double, kStateDim, kNoiseDim>::Zero();
        Fi.block<12, kNoiseDim>(3, 0) = Eigen::Matrix<double, 12, kNoiseDim>::Identity();

        // Qi
        Eigen::Matrix<double, kNoiseDim, kNoiseDim> Qi = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Zero();
        Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt2 * acc_noise_;
        Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt2 * gyr_noise_;
        Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
        Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyr_bias_noise_;

        // P: error-state covariance
        state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
    }

    void update_6DOF(const Eigen::Vector3d& measured_position, const Eigen::Matrix3d& measured_orientation, const Eigen::Matrix3d& residual_orientation,\
                        const Eigen::Matrix3d& cov, const Eigen::Vector3d& gnss_position_imuFrame, const Eigen::Matrix3d& gnss_orientation_imuFrame) {
        Eigen::Vector3d predicted_position = state_ptr_->p_GI;
        Eigen::Vector3d residual_position = measured_position - predicted_position;
        // jacobian
        Eigen::Matrix<double, 6, 15> H;
        H.setZero();

        Eigen::Isometry3d T_predict, T_identity, T_gnss_imuFrame;
        T_identity = Eigen::Isometry3d::Identity();
        T_predict.linear() = state_ptr_->r_GI;
        T_predict.translation() = state_ptr_->p_GI;
        T_gnss_imuFrame.linear() = gnss_orientation_imuFrame;
        T_gnss_imuFrame.translation() = gnss_position_imuFrame;
        Eigen::Isometry3d T_measured;
        T_measured.linear() = measured_orientation;
        T_measured.translation() = measured_position;
        

        Eigen::Quaterniond measured_q(T_measured.linear());

        const Eigen::Matrix3d &R_identity = T_identity.linear();

        Eigen::Quaterniond q_identity(R_identity);
        Eigen::Quaterniond q_gnss_imu_frame(gnss_orientation_imuFrame);
        Eigen::Quaterniond q_predict(T_predict.linear());    


        // // case1
        // Eigen::Matrix<double, 6, 15> H2;
        // H2.setZero();
        // H2.block<3, 3>(0, 0) = R_identity;
        // H2.block<3, 3>(0, 6) = -R_identity * T_predict.linear() * skew_matrix(T_gnss_imuFrame.inverse().translation());
        // H2.block<3, 3>(3, 6) = (quat_left_matrix((q_identity * q_predict).normalized()) * quat_right_matrix(q_gnss_imu_frame.conjugate())).block<3, 3>(1, 1);
        // H.block<3, 3>(0, 0) = R_identity;
        // H.block<3, 3>(0, 6) = -R_identity * T_predict.linear() * skew_matrix(T_gnss_imuFrame.inverse().translation());
        // H.block<3, 3>(3, 6) = (quat_left_matrix((q_identity * q_predict).normalized()) * quat_right_matrix(q_gnss_imu_frame.conjugate())).block<3, 3>(1, 1);
        

        // // case2
        Eigen::Matrix4d m4;
        H.block<3, 3>(0, 0) = -R_identity;
        H.block<3, 3>(0, 6) = R_identity * T_predict.linear() * skew_matrix(T_gnss_imuFrame.inverse().translation());
        m4 = quat_left_matrix((measured_q.conjugate() * q_identity * q_predict).normalized()) * quat_right_matrix(q_gnss_imu_frame.conjugate());
        H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
        H *= -1.0;

        // std::cout << H << std::endl;
        // std::cout << H2 << std::endl;


        // Create the full 6x6 covariance matrix
        Eigen::Matrix<double, 6, 6> full_cov;
        full_cov.setZero();

        // Assign the position (translation) covariance
        full_cov.block<3, 3>(0, 0) = cov;

        // Assign the rotation covariance (this is a simple identity matrix times a scalar, you should replace this with the appropriate covariance)
        double rotation_cov_value = 1e-2; // Example value, you should set this based on your application
        full_cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * rotation_cov_value;


        update_6dof_measurement(H, full_cov, residual_orientation, residual_position);
    }

    void update_3DOF(const Eigen::Vector3d& measured_position, const Eigen::Matrix3d& cov, const Eigen::Vector3d& gnss_position_imuFrame) {

        Eigen::Vector3d predicted_position = state_ptr_->p_GI;
        Eigen::Vector3d residual = measured_position - predicted_position;
        // double abs_residual = residual.norm();
        // jacobian
        Eigen::Matrix<double, 3, 15> H;
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 6) = -state_ptr_->r_GI * skew_matrix(gnss_position_imuFrame);

        // measurement covariance
        const Eigen::Matrix3d &V = cov;

        update_3dof_measurement(H, V, residual);
    }
    /**
     * @brief measurement update procedure
     * @ref ESKF 6: Fusing IMU with complementary sensory data
     * @tparam H_type 
     * @tparam Res_type 
     * @tparam R_type 
     * @param H 
     * @param V 
     * @param r 
     */
    template <class H_type, class Res_type, class R_type>
    void update_3dof_measurement(
        const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &V, const Eigen::MatrixBase<Res_type> &r) {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
        EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

        // compute K
        const MatrixSD &P = state_ptr_->cov;
        const R_type S = H * P * H.transpose() + V;
        const Eigen::Matrix<double, kStateDim, R_type::RowsAtCompileTime> K = P * H.transpose() * S.inverse();

        // delta_x
        const Eigen::Matrix<double, kStateDim, 1> delta_x = K * r;

        // update: nominal-state + observation error
        state_ptr_->p_GI += delta_x.block<3, 1>(0, 0);
        state_ptr_->v_GI += delta_x.block<3, 1>(3, 0);
        const Eigen::Vector3d &dR = delta_x.block<3, 1>(6, 0);
        if (dR.norm() > DBL_EPSILON) {
            state_ptr_->r_GI *= Eigen::AngleAxisd(dR.norm(), dR.normalized()).toRotationMatrix();
        }
        state_ptr_->acc_bias += delta_x.block<3, 1>(9, 0);
        state_ptr_->gyr_bias += delta_x.block<3, 1>(12, 0);

        // update: error-state covariance
        const MatrixSD I_KH = MatrixSD::Identity() - K * H;
        state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
    }

    void update_6dof_measurement(
        const Eigen::Matrix<double, 6, 15> &H,
        const Eigen::Matrix<double, 6, 6> &V,
        const Eigen::Matrix3d &residual_orientation,
        const Eigen::Vector3d &residual_position) {

        // EIGEN_STATIC_ASSERT_FIXED_SIZE(H);
        // EIGEN_STATIC_ASSERT_FIXED_SIZE(V);

        // compute K
        const MatrixSD &P = state_ptr_->cov;
        Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + V;
        Eigen::Matrix<double, kStateDim, 6> K = P * H.transpose() * S.inverse();

        // residual vector (6x1)
        Eigen::Matrix<double, 6, 1> r;
        r.block<3, 1>(0, 0) = residual_position;

        Eigen::Quaterniond q_res(residual_orientation);
        r.block<3, 1>(3, 0) = 2.0 * q_res.vec() / q_res.w();

        // delta_x
        Eigen::Matrix<double, kStateDim, 1> delta_x = K * r;

        // update: nominal-state + observation error
        state_ptr_->p_GI += delta_x.block<3, 1>(0, 0); // Position update
        state_ptr_->v_GI += delta_x.block<3, 1>(3, 0); // Velocity update
        Eigen::Vector3d dR = delta_x.block<3, 1>(6, 0); // Rotation vector
        if (dR.norm() > DBL_EPSILON) {
            state_ptr_->r_GI *= Eigen::AngleAxisd(dR.norm(), dR.normalized()).toRotationMatrix(); // Update rotation matrix
        }
        state_ptr_->acc_bias += delta_x.block<3, 1>(9, 0); // Accelerometer bias update
        state_ptr_->gyr_bias += delta_x.block<3, 1>(12, 0); // Gyroscope bias update

        // update: error-state covariance
        Eigen::Matrix<double, kStateDim, kStateDim> I_KH = MatrixSD::Identity() - K * H;
        state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
    }

    ~KF() {}

   private:
    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;

    double sigma_posvel_;
    double sigma_rolpit_deg_;
    double sigma_yaw_deg_;
    
};

using KFPtr = std::unique_ptr<KF>;

