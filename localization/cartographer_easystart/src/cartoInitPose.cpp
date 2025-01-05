#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>

class CartographerInitPose : public rclcpp::Node {
public:
    CartographerInitPose() : Node("carto_init_pose") {
        // Subscribe to the /initialpose topic
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&CartographerInitPose::initialPoseCallback, this, std::placeholders::_1));

        // Create a client for the /start_trajectory service
        start_trajectory_client_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>(
            "/start_trajectory");

        RCLCPP_INFO(this->get_logger(), "CartographerInitPose node is ready.");
    }

private:
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received initial pose: [%f, %f, %f]",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.orientation.z);

        // Wait for the service to be available
        if (!start_trajectory_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service /start_trajectory not available!");
            return;
        }

        // Create a service request
        auto request = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
        request->configuration_directory = "/path/to/configuration_directory"; // Update to actual path
        request->configuration_basename = "configuration_file.lua";            // Update to actual file name
        request->use_initial_pose = true;
        request->initial_pose.position.x = msg->pose.pose.position.x;
        request->initial_pose.position.y = msg->pose.pose.position.y;
        request->initial_pose.position.z = 0.0; // Assuming 2D setup
        request->initial_pose.orientation = msg->pose.pose.orientation;
        request->relative_to_trajectory_id = 0; // Modify based on your use case

        // Send the service request
        auto future = start_trajectory_client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->status.code == 0) {
                RCLCPP_INFO(this->get_logger(), "Successfully started trajectory with ID: %d", response->trajectory_id);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to start trajectory: %s", response->status.message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /start_trajectory");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartographerInitPose>());
    rclcpp::shutdown();
    return 0;
}
