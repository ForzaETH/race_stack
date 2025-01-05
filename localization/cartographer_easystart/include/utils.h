#pragma once

#include <Eigen/Core>

constexpr int kStateDim = 15;
constexpr int kNoiseDim = 12;
using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

 struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // error-state
        MatrixSD cov;

        // nominal-state
        Eigen::Vector3d p_GI;
        Eigen::Vector3d v_GI;
        Eigen::Vector3d av_GI;
        Eigen::Matrix3d r_GI;
        Eigen::Vector3d acc_bias;
        Eigen::Vector3d gyr_bias;

        double timestamp;

        State() {
            cov.setZero();

            p_GI.setZero();
            v_GI.setZero();
            r_GI.setIdentity();

            acc_bias.setZero();
            gyr_bias.setZero();
        }
    };
using StatePtr = std::shared_ptr<State>;


struct ImuData {
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};
using ImuDataPtr = std::shared_ptr<ImuData>;
using ImuDataConstPtr = std::shared_ptr<const ImuData>;

struct gnssData {
    double timestamp;

    Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter
    Eigen::Vector3d xyz_enu;  // 
    Eigen::Matrix3d cov;  // Covariance in m^2
};
using gnssDataPtr = std::shared_ptr<gnssData>;
using gnssDataConstPtr = std::shared_ptr<const gnssData>;

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return w;
}

  static Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
  }

  static Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = -skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
  }
