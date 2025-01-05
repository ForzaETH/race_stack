#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <deque>

class Imu {
    public:
        bool is_initialized;
        Eigen::Vector3d globalZaxis_imuFrame;

    Imu() = delete;

    Imu(const Imu &) = delete;

    explicit Imu(int kImuBufSize, double acc_std_threshold)
        : ImuBufSize_(kImuBufSize), acc_std_threshold_(acc_std_threshold) {
        is_initialized = false;
        imu_buf_.clear();
        globalZaxis_imuFrame.setZero();

    }

    void initializer(ImuDataPtr imu_ptr) {
        imu_buf_.push_back(imu_ptr);

        if (imu_buf_.size() > ImuBufSize_) imu_buf_.pop_front();

        if(imu_buf_.size() == ImuBufSize_){
            computeGeocentricVectorFromIMU();
        }
    }

    void computeGeocentricVectorFromIMU() {
        // mean and std of IMU accs
        Eigen::Vector3d sum_acc(0., 0., 0.);
        for (const auto imu_data : imu_buf_) {
            sum_acc += imu_data->acc;
        }
        const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_.size();
        // printf("[gnss_imu %s] mean_acc: (%f, %f, %f)!!!\n", __FUNCTION__, mean_acc[0], mean_acc[1], mean_acc[2]);
        // std::cout << (mean_acc.cwiseAbs2()).cwiseSqrt() <<std::endl;
        Eigen::Vector3d sum_err2(0., 0., 0.);
        for (const auto imu_data : imu_buf_) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
        const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf_.size()).cwiseSqrt();
        // acc std limit
        if (std_acc.maxCoeff() > acc_std_threshold_) {
            printf("[IMU %s] Too big acc std: (%f, %f, %f)!!!\n", __FUNCTION__, std_acc[0], std_acc[1], std_acc[2]);
            return;
        }
        // z-axis
        const Eigen::Vector3d &mean_acc_norm = mean_acc.normalized();
        globalZaxis_imuFrame = mean_acc_norm;
        is_initialized = true;
        return;
    }

    ~Imu() {}

    private:
        bool is_dual_;
        double acc_std_threshold_;
        int ImuBufSize_;
        std::deque<ImuDataConstPtr> imu_buf_;

};

using ImuPtr = std::unique_ptr<Imu>;
