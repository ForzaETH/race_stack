#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kalman_filter_node") {
        // Initialize state vector and covariance matrices
        x_ = Eigen::Vector4d::Zero();
        P_ = Eigen::Matrix4d::Identity();
        F_ = Eigen::Matrix4d::Identity();
        Q_ = Eigen::Matrix4d::Identity() * 0.01;
        Q_.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * 0.3;

        // Observation matrix
        H_ = Eigen::MatrixXd::Zero(2, 4);
        H_.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();

        // Measurement noise covariance
        R_ = Eigen::Matrix2d::Identity() * 0.0001;

        // ROS2 Subscriptions and Publishers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "tracked_pose", 100, std::bind(&KalmanFilterNode::pose_callback, this, std::placeholders::_1)
        );

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable); // 신뢰성 보장
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("tracked_odom", qos);
                // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("tracked_odom", 10);
    }

private:
    // State variables
    Eigen::Vector4d x_;
    Eigen::Matrix4d P_, F_, Q_;
    Eigen::MatrixXd H_;
    Eigen::Matrix2d R_;
    rclcpp::Time last_time_;

    // ROS2 subscription and publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 측정값: x와 y만 반영
        Eigen::Vector2d z(msg->pose.position.x, msg->pose.position.y);

        // Delta time 계산
        double dt = get_delta_time();
        if (dt < 0.0) return; // 첫 데이터 수신 시 건너뜀

        // 상태 전이 행렬 업데이트
        update_state_transition(dt);

        // Prediction 단계
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // Update 단계
        Eigen::Vector2d y = z - H_ * x_;
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ += K * y;
        P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;

        // Odometry 메시지 생성 및 게시
        publish_odometry(msg);
    }

    void update_state_transition(double dt) {
        F_.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * dt;
    }

    double get_delta_time() {
        auto now = this->get_clock()->now();
        if (last_time_.nanoseconds() == 0) {
            last_time_ = now;
            return -1.0; // Skip first iteration
        }

        double dt = (now - last_time_).seconds();
        last_time_ = now;
        return dt;
    }

    void publish_odometry(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

        // PoseStamped의 frame_id를 Odometry에 반영
        odom_msg->header.stamp = this->get_clock()->now();
        odom_msg->header.frame_id = msg->header.frame_id;

        // 필터링된 위치
        odom_msg->pose.pose.position.x = x_(0);
        odom_msg->pose.pose.position.y = x_(1);
        odom_msg->pose.pose.position.z = msg->pose.position.z; // z 값 그대로 사용

        // Orientation 그대로 복사
        odom_msg->pose.pose.orientation = msg->pose.orientation;
        // Orientation에서 yaw 추출
        double roll, pitch, yaw;
        tf2::Quaternion quat(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // 필터링된 속도
        // odom_msg->twist.twist.linear.x = x_(2);
        // odom_msg->twist.twist.linear.y = x_(3);
        // odom_msg->twist.twist.linear.z = 0.0;
        // Global velocity (x_, y_) -> Local velocity (vx, vy)
        double vx_local = x_(2) * cos(yaw) + x_(3) * sin(yaw);
        double vy_local = -x_(2) * sin(yaw) + x_(3) * cos(yaw);

        // 필터링된 속도
        odom_msg->twist.twist.linear.x = vx_local;
        odom_msg->twist.twist.linear.y = vy_local;
        odom_msg->twist.twist.linear.z = 0.0;
        // 공분산: 위치
        std::array<double, 36> pose_covariance = {0};
        pose_covariance[0] = P_(0, 0); // x-x 공분산
        pose_covariance[1] = P_(0, 1); // x-y 공분산
        pose_covariance[6] = P_(1, 0); // y-x 공분산
        pose_covariance[7] = P_(1, 1); // y-y 공분산
        pose_covariance[35] = 0.1; // z 방향은 임의값
        odom_msg->pose.covariance = pose_covariance;

        // 공분산: 속도
        std::array<double, 36> twist_covariance = {0};
        twist_covariance[0] = P_(2, 2); // vx-vx 공분산
        twist_covariance[1] = P_(2, 3); // vx-vy 공분산
        twist_covariance[6] = P_(3, 2); // vy-vx 공분산
        twist_covariance[7] = P_(3, 3); // vy-vy 공분산
        odom_msg->twist.covariance = twist_covariance;

        // 메시지 게시
        odom_pub_->publish(*odom_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
