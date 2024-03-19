#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <f110_msgs/WpntArray.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <iomanip>
#include <random>

class LaserMapOverlapChecker {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_, laser_sub_, trackbounds_sub_, gb_sub_;
    ros::Publisher debug_lidar_pub_, alignment_score_pub_, alignment_textmrk_pub_;
    tf::TransformListener tf_listener_;
    nav_msgs::OccupancyGrid map_;
    visualization_msgs::MarkerArray trackbounds_;
    geometry_msgs::Point start_pos_;
    sensor_msgs::LaserScan scan_;
    bool debug_flag_;
    int downsample_factor_;
    double threshold_;
    int frequency_;
    bool scan_initialized_;
    bool trackbounds_initialized_;
    bool start_pos_initialized_;

public:
    LaserMapOverlapChecker() : nh_("~") {
        // Initialize parameters
        nh_.param("debug", debug_flag_, true);
        nh_.param("downsample_factor", downsample_factor_, 20);
        nh_.param("threshold", threshold_, 0.1);
        nh_.param("frequency", frequency_, 5);

        // Subscribers
        map_sub_ = nh_.subscribe("/map", 10, &LaserMapOverlapChecker::mapCallback, this);
        laser_sub_ = nh_.subscribe("/scan", 10, &LaserMapOverlapChecker::laserCallback, this);
        trackbounds_sub_ = nh_.subscribe("/trackbounds/markers", 10, &LaserMapOverlapChecker::trackboundsCallback, this);
        gb_sub_ = nh_.subscribe("/global_waypoints", 10, &LaserMapOverlapChecker::glbWpntsCallback, this);

        // Publishers
        debug_lidar_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/scanalign/debug_lidar", 1);
        alignment_score_pub_ = nh_.advertise<std_msgs::Float32>("/scanalign/alignment_score", 1);
        alignment_textmrk_pub_ = nh_.advertise<visualization_msgs::Marker>("/scanalign/alignment_textmrk", 1);

        //misc
        scan_initialized_ = false;
        trackbounds_initialized_ = false;
        start_pos_initialized_ = false;

        // Main loop
        loop();
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_ = *msg;
        scan_initialized_ = true;
    }

    void trackboundsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        trackbounds_ = *msg;
        trackbounds_initialized_ = true;
    }

    void glbWpntsCallback(const f110_msgs::WpntArray::ConstPtr& msg) {
        std::vector<f110_msgs::Wpnt> glb_wpnts = msg->wpnts;
        if (!start_pos_initialized_) {
            geometry_msgs::Point p0;
            p0.x = glb_wpnts[0].x_m;
            p0.y = glb_wpnts[0].y_m;

            geometry_msgs::Point p1;
            p1.x = glb_wpnts[1].x_m;
            p1.y = glb_wpnts[1].y_m;
            
            // Compute a vector from p0 to p1
            double dx = p1.x - p0.x;
            double dy = p1.y - p0.y;
            
            // Compute a normal vector, rotated 90 degrees counter-clockwise
            double nx = -dy;
            double ny = dx;
            
            // Normalize the normal vector
            double norm = std::sqrt(nx*nx + ny*ny);
            nx /= norm;
            ny /= norm;
            
            // Compute the position of the text marker, 1.5 units to the left of the first waypoint
            start_pos_.x = p0.x + 1.5 * nx;
            start_pos_.y = p0.y + 1.5 * ny;
            start_pos_.z = 0.0;  
            start_pos_initialized_ = true;
        }
        else{
            // Do nothing
        }
    }


    //Main Loop
    void loop() {
        ROS_INFO("[Scan Analyzer] Starting scan alignment analyzer");
        ros::Rate rate(frequency_);
        while (ros::ok()) {
            //measure time for loop
            //ros::Time start = ros::Time::now();

            if (scan_initialized_ && trackbounds_initialized_) {
                Eigen::MatrixXd xyz_mf = laser2mapConversion(scan_);
                double alignment = computeScanAlignment(xyz_mf, trackbounds_);

                //ROS_INFO("Alignment score: %f", alignment);
                std_msgs::Float32 alignment_msg;
                alignment_msg.data = alignment;
                alignment_score_pub_.publish(alignment_msg);

                //Publish the marker
                if (start_pos_initialized_){
                    publishAlignmentScoreText(alignment);
                }
                
                // Publish the debug markers if wanted
                if (debug_flag_) {
                    publishDebugLidarPoints(xyz_mf, scan_.header.stamp);
                }
            }
            else{
                ROS_INFO("[Scan Analyzer] Scan alignment analyzer not initialized");
            }
            
            //measure time for loop
            // ros::Time end = ros::Time::now();
            // ros::Duration elapsed = end - start;
            // float frequency = 1.0 / elapsed.toSec();
            // ROS_INFO("[Scan Analyzer] Loop time: %f, Possible Frequency: %f", elapsed.toSec(), frequency);

            ros::spinOnce();
            rate.sleep();
        }
    }

    Eigen::MatrixXd laser2mapConversion(const sensor_msgs::LaserScan& laser) {
        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("/map", "/laser", ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return Eigen::MatrixXd();  // Return an empty matrix
        }

        Eigen::Affine3d T;
        tf::transformTFToEigen(transform, T);

        // Parameters for Gaussian distribution
        const int num_points_to_select = laser.ranges.size() / downsample_factor_;  
        int middle_index = laser.ranges.size() / 2;
        double std_dev = 0.68 * num_points_to_select;  // Standard deviation for the normal distribution

        // Random engine and normal distribution
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(middle_index, std_dev);

        std::vector<double> valid_ranges;
        std::vector<double> valid_angles;

        for (int i = 0; i < num_points_to_select; ++i) {
            int index = static_cast<int>(distribution(generator));

            // Ensure index is within valid bounds
            index = std::max(0, std::min(static_cast<int>(laser.ranges.size()) - 1, index));

            if (laser.ranges[index] > 0.1) {  // Filtering out ranges that are under 0.1 meters
                valid_ranges.push_back(laser.ranges[index]);
                valid_angles.push_back(laser.angle_min + index * laser.angle_increment);
            }
        }

        // Convert laser polar coordinates to Cartesian
        Eigen::MatrixXd cartesian_points(4, valid_ranges.size());  // 4xN matrix
        for (size_t i = 0; i < valid_ranges.size(); ++i) {
            double angle = valid_angles[i];
            cartesian_points(0, i) = valid_ranges[i] * cos(angle);  // x-coordinate
            cartesian_points(1, i) = valid_ranges[i] * sin(angle);  // y-coordinate
            cartesian_points(2, i) = 0.0;  // z-coordinate
            cartesian_points(3, i) = 1.0;  // homogeneous coordinate
        }
        
        // Apply the transformation
        Eigen::MatrixXd transformed_points = T.matrix() * cartesian_points;
        return transformed_points.topRows(3);  // Return only the x, y, z coordinates
    }

    double computeScanAlignment(const Eigen::MatrixXd& scan_in_map, const visualization_msgs::MarkerArray& trackbounds) {
        Eigen::MatrixXd boundary_positions(trackbounds.markers.size(), 3);
        for (size_t i = 0; i < trackbounds.markers.size(); ++i) {
            boundary_positions(i, 0) = trackbounds.markers[i].pose.position.x;
            boundary_positions(i, 1) = trackbounds.markers[i].pose.position.y;
            boundary_positions(i, 2) = 0.0;
        }

        int align_cnt = 0;
        double threshold_squared = threshold_ * threshold_;

        for (int i = 0; i < scan_in_map.cols(); ++i) {
            Eigen::VectorXd scan_point = scan_in_map.col(i);
            for (int j = 0; j < boundary_positions.rows(); ++j) {
                Eigen::VectorXd boundary_point = boundary_positions.row(j);
                double distance_squared = (scan_point - boundary_point).squaredNorm();

                if (distance_squared < threshold_squared) {
                    align_cnt++;
                    break;
                }
            }
        }

        double align_score = static_cast<double>(align_cnt) / (1080.0/downsample_factor_) * 100.0;
        return align_score;
    }

    // MISC Functions
    void publishDebugLidarPoints(const Eigen::MatrixXd& points, const ros::Time& timestamp) {
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < points.cols(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = timestamp;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = points(0, i);
        marker.pose.position.y = points(1, i);
        marker.pose.position.z = points(2, i);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.6;
        markers.markers.push_back(marker);
    }
    debug_lidar_pub_.publish(markers);
}

    void publishAlignmentScoreText(double score) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = start_pos_.x;
        marker.pose.position.y = start_pos_.y;
        marker.pose.position.z = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.5;  // Text size
        marker.color.a = 1.0;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;

        // Create a string stream and use fixed and setprecision to format the score
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2) << score;
        marker.text = "Scan Alignment Score: " + stream.str() + " [%]";

        alignment_textmrk_pub_.publish(marker);
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_alignment_analyzer");
    LaserMapOverlapChecker checker;
    return 0;
}
