#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class SlamInterface : public rclcpp::Node
{
public:
    SlamInterface() : Node("slam_interface")
    {
        // Load all parameters from config files
        loadParameters();
        
        // Create state estimation publisher
        state_estimation_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/state_estimation", 10);

        // Create marker publisher for box filter visualization
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/body_box_filter", 10);

        RCLCPP_INFO(this->get_logger(), "SLAM Interface initialized - publishing state_estimation and box filter markers");
    }

private:
    void loadParameters()
    {
        // All parameters are loaded from YAML config files via ROS2 parameter system
        // The YAML files are loaded in the launch file using the --ros-args --params-file syntax
        
        // Frame names for static transforms
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("init_frame", "lidar_init");
        this->declare_parameter<std::string>("vehicle_frame", "vehicle");
        this->declare_parameter<std::string>("laser_imu_frame", "laser");
        this->declare_parameter<std::string>("laser_frame", "laser");
        this->declare_parameter<std::string>("init_odom_frame", "init_odom");
        
        // T_v_i: Transform from IMU to vehicle frame (new convention)
        this->declare_parameter<double>("t_v_i.x", 0.0);
        this->declare_parameter<double>("t_v_i.y", 0.0);
        this->declare_parameter<double>("t_v_i.z", 0.0);
        this->declare_parameter<std::vector<double>>("R_v_i.data", 
            std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        
        // T_i_l: Transform from laser to IMU frame (laser extrinsic)
        this->declare_parameter<double>("t_i_l.x", 0.0);
        this->declare_parameter<double>("t_i_l.y", 0.0);
        this->declare_parameter<double>("t_i_l.z", 0.0);
        this->declare_parameter<std::vector<double>>("R_i_l.data", 
            std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        
        // Backward compatibility: old parameter names
        this->declare_parameter<double>("translation.x", 0.0);
        this->declare_parameter<double>("translation.y", 0.0);
        this->declare_parameter<double>("translation.z", 0.0);
        this->declare_parameter<std::vector<double>>("rotation_matrix.data", 
            std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        
        // Box filter parameters
        this->declare_parameter<bool>("filter_enabled", true);
        this->declare_parameter<double>("box_filter.min_x", -0.3);
        this->declare_parameter<double>("box_filter.max_x", 0.0);
        this->declare_parameter<double>("box_filter.min_y", -0.3);
        this->declare_parameter<double>("box_filter.max_y", 0.3);
        this->declare_parameter<double>("box_filter.min_z", 0.0);
        this->declare_parameter<double>("box_filter.max_z", 2.0);
        
        // Topics
        this->declare_parameter<std::string>("input_topic", "/registered_scan");
        this->declare_parameter<std::string>("output_topic", "/registered_scan_filtered_map");
        
        // Get parameters
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("init_frame", init_frame_);
        this->get_parameter("vehicle_frame", vehicle_frame_);
        this->get_parameter("laser_imu_frame", laser_imu_frame_);
        this->get_parameter("laser_frame", laser_frame_);
        this->get_parameter("init_odom_frame", init_odom_frame_);
        
        // Load T_v_i (vehicle <- IMU transform) with backward compatibility
        // Try new convention first (t_v_i), fall back to old (translation)
        if (this->has_parameter("t_v_i.x")) {
            this->get_parameter("t_v_i.x", T_v_i_x_);
            this->get_parameter("t_v_i.y", T_v_i_y_);
            this->get_parameter("t_v_i.z", T_v_i_z_);
        } else {
            this->get_parameter("translation.x", T_v_i_x_);
            this->get_parameter("translation.y", T_v_i_y_);
            this->get_parameter("translation.z", T_v_i_z_);
        }
        
        // Load R_v_i rotation matrix with backward compatibility
        std::vector<double> R_v_i_data;
        if (this->has_parameter("R_v_i.data")) {
            this->get_parameter("R_v_i.data", R_v_i_data);
        } else {
            this->get_parameter("rotation_matrix.data", R_v_i_data);
        }
        if (R_v_i_data.size() != 9) {
            RCLCPP_ERROR(this->get_logger(), 
                "R_v_i.data must contain exactly 9 values, got %zu", R_v_i_data.size());
            throw std::runtime_error("Invalid rotation matrix size");
        }
        R_v_i_11_ = R_v_i_data[0]; R_v_i_12_ = R_v_i_data[1]; R_v_i_13_ = R_v_i_data[2];
        R_v_i_21_ = R_v_i_data[3]; R_v_i_22_ = R_v_i_data[4]; R_v_i_23_ = R_v_i_data[5];
        R_v_i_31_ = R_v_i_data[6]; R_v_i_32_ = R_v_i_data[7]; R_v_i_33_ = R_v_i_data[8];
        
        // Load T_i_l (IMU <- laser transform, i.e., laser extrinsic)
        this->get_parameter("t_i_l.x", T_i_l_x_);
        this->get_parameter("t_i_l.y", T_i_l_y_);
        this->get_parameter("t_i_l.z", T_i_l_z_);
        
        std::vector<double> R_i_l_data;
        this->get_parameter("R_i_l.data", R_i_l_data);
        if (R_i_l_data.size() != 9) {
            RCLCPP_ERROR(this->get_logger(), 
                "R_i_l.data must contain exactly 9 values, got %zu", R_i_l_data.size());
            throw std::runtime_error("Invalid rotation matrix size for R_i_l");
        }
        R_i_l_11_ = R_i_l_data[0]; R_i_l_12_ = R_i_l_data[1]; R_i_l_13_ = R_i_l_data[2];
        R_i_l_21_ = R_i_l_data[3]; R_i_l_22_ = R_i_l_data[4]; R_i_l_23_ = R_i_l_data[5];
        R_i_l_31_ = R_i_l_data[6]; R_i_l_32_ = R_i_l_data[7]; R_i_l_33_ = R_i_l_data[8];
        
        this->get_parameter("filter_enabled", filter_enabled_);
        this->get_parameter("box_filter.min_x", box_min_x_);
        this->get_parameter("box_filter.max_x", box_max_x_);
        this->get_parameter("box_filter.min_y", box_min_y_);
        this->get_parameter("box_filter.max_y", box_max_y_);
        this->get_parameter("box_filter.min_z", box_min_z_);
        this->get_parameter("box_filter.max_z", box_max_z_);
        
        std::string input_topic, output_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);
        
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize static transform broadcaster and publish transforms
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishStaticTransforms();

        // Create subscriber and publisher
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&SlamInterface::pointCloudCallback, this, std::placeholders::_1));
        
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic, 10);

        state_estimation_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz = 20 ms period
            std::bind(&SlamInterface::publishStateEstimation, this));

        // Print configuration
        RCLCPP_INFO(this->get_logger(), "SLAM Interface - PointCloud Preprocessor initialized");
        RCLCPP_INFO(this->get_logger(), "Static transforms published:");
        RCLCPP_INFO(this->get_logger(), "  1) %s -> %s", map_frame_.c_str(), init_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  2) %s -> %s (same as 1 at startup)", map_frame_.c_str(), init_odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  3) %s -> %s", laser_imu_frame_.c_str(), vehicle_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  4) %s -> %s", laser_imu_frame_.c_str(), laser_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  T_v_i (vehicle <- IMU): [%.3f, %.3f, %.3f]", 
                    T_v_i_x_, T_v_i_y_, T_v_i_z_);
        RCLCPP_INFO(this->get_logger(), "  R_v_i:");
        RCLCPP_INFO(this->get_logger(), "    [%.6f  %.6f  %.6f]", 
                    R_v_i_11_, R_v_i_12_, R_v_i_13_);
        RCLCPP_INFO(this->get_logger(), "    [%.6f  %.6f  %.6f]", 
                    R_v_i_21_, R_v_i_22_, R_v_i_23_);
        RCLCPP_INFO(this->get_logger(), "    [%.6f  %.6f  %.6f]", 
                    R_v_i_31_, R_v_i_32_, R_v_i_33_);
        RCLCPP_INFO(this->get_logger(), "  T_i_l (IMU <- laser): [%.6f, %.6f, %.6f]", 
                    T_i_l_x_, T_i_l_y_, T_i_l_z_);
        RCLCPP_INFO(this->get_logger(), "  R_i_l:");
        RCLCPP_INFO(this->get_logger(), "    [%.6f  %.6f  %.6f]", 
                    R_i_l_11_, R_i_l_12_, R_i_l_13_);
        RCLCPP_INFO(this->get_logger(), "    [%.6f  %.6f  %.6f]", 
                    R_i_l_21_, R_i_l_22_, R_i_l_23_);
        RCLCPP_INFO(this->get_logger(), "    [%.6f  %.6f  %.6f]", 
                    R_i_l_31_, R_i_l_32_, R_i_l_33_);
        RCLCPP_INFO(this->get_logger(), "Filtering box in vehicle frame:");
        RCLCPP_INFO(this->get_logger(), "  X: [%.2f, %.2f]", box_min_x_, box_max_x_);
        RCLCPP_INFO(this->get_logger(), "  Y: [%.2f, %.2f]", box_min_y_, box_max_y_);
        RCLCPP_INFO(this->get_logger(), "  Z: [%.2f, %.2f]", box_min_z_, box_max_z_);
        RCLCPP_INFO(this->get_logger(), "Filter enabled: %s", filter_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "State estimation: %s -> %s", map_frame_.c_str(), vehicle_frame_.c_str());
    }

    void publishStaticTransforms()
    {
        auto stamp = this->get_clock()->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // Transform 1: map -> init_frame (using T_v_i)
        tf2::Matrix3x3 R_v_i(
            R_v_i_11_, R_v_i_12_, R_v_i_13_,
            R_v_i_21_, R_v_i_22_, R_v_i_23_,
            R_v_i_31_, R_v_i_32_, R_v_i_33_
        );
        tf2::Quaternion q_v_i;
        R_v_i.getRotation(q_v_i);

        geometry_msgs::msg::TransformStamped transform_map_camera;
        transform_map_camera.header.stamp = stamp;
        transform_map_camera.header.frame_id = map_frame_;
        transform_map_camera.child_frame_id = init_frame_;
        transform_map_camera.transform.translation.x = T_v_i_x_;
        transform_map_camera.transform.translation.y = T_v_i_y_;
        transform_map_camera.transform.translation.z = T_v_i_z_;
        transform_map_camera.transform.rotation.x = q_v_i.x();
        transform_map_camera.transform.rotation.y = q_v_i.y();
        transform_map_camera.transform.rotation.z = q_v_i.z();
        transform_map_camera.transform.rotation.w = q_v_i.w();
        transforms.push_back(transform_map_camera);

        // Transform 1b: map -> init_odom_frame (same as map -> init_frame at startup)
        geometry_msgs::msg::TransformStamped transform_map_init_odom;
        transform_map_init_odom.header.stamp = stamp;
        transform_map_init_odom.header.frame_id = map_frame_;
        transform_map_init_odom.child_frame_id = init_odom_frame_;
        transform_map_init_odom.transform = transform_map_camera.transform;
        transforms.push_back(transform_map_init_odom);

        // Transform 2: laser_imu_frame -> vehicle (inverse of T_v_i)
        tf2::Quaternion q_i_v = q_v_i.inverse();
        tf2::Vector3 t_v_i(T_v_i_x_, T_v_i_y_, T_v_i_z_);
        tf2::Vector3 t_i_v = tf2::quatRotate(q_i_v, -t_v_i);
        
        geometry_msgs::msg::TransformStamped transform_imu_vehicle;
        transform_imu_vehicle.header.stamp = stamp;
        transform_imu_vehicle.header.frame_id = laser_imu_frame_;
        transform_imu_vehicle.child_frame_id = vehicle_frame_;
        transform_imu_vehicle.transform.translation.x = t_i_v.x();
        transform_imu_vehicle.transform.translation.y = t_i_v.y();
        transform_imu_vehicle.transform.translation.z = t_i_v.z();
        transform_imu_vehicle.transform.rotation.x = q_i_v.x();
        transform_imu_vehicle.transform.rotation.y = q_i_v.y();
        transform_imu_vehicle.transform.rotation.z = q_i_v.z();
        transform_imu_vehicle.transform.rotation.w = q_i_v.w();
        transforms.push_back(transform_imu_vehicle);
        
        // Transform 3: laser_imu_frame -> laser_frame (using T_i_l)
        tf2::Matrix3x3 R_i_l(
            R_i_l_11_, R_i_l_12_, R_i_l_13_,
            R_i_l_21_, R_i_l_22_, R_i_l_23_,
            R_i_l_31_, R_i_l_32_, R_i_l_33_
        );
        tf2::Quaternion q_i_l;
        R_i_l.getRotation(q_i_l);
        
        geometry_msgs::msg::TransformStamped transform_imu_laser;
        transform_imu_laser.header.stamp = stamp;
        transform_imu_laser.header.frame_id = laser_imu_frame_;
        transform_imu_laser.child_frame_id = laser_frame_;
        transform_imu_laser.transform.translation.x = T_i_l_x_;
        transform_imu_laser.transform.translation.y = T_i_l_y_;
        transform_imu_laser.transform.translation.z = T_i_l_z_;
        transform_imu_laser.transform.rotation.x = q_i_l.x();
        transform_imu_laser.transform.rotation.y = q_i_l.y();
        transform_imu_laser.transform.rotation.z = q_i_l.z();
        transform_imu_laser.transform.rotation.w = q_i_l.w();
        transforms.push_back(transform_imu_laser);
        
        // Publish all transforms
        static_tf_broadcaster_->sendTransform(transforms);
        
        RCLCPP_INFO(this->get_logger(),
                    "Published static transforms: %s->%s, %s->%s, %s->%s, and %s->%s",
                    map_frame_.c_str(), init_frame_.c_str(),
                    map_frame_.c_str(), init_odom_frame_.c_str(),
                    laser_imu_frame_.c_str(), vehicle_frame_.c_str(),
                    laser_imu_frame_.c_str(), laser_frame_.c_str());
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // If filter is disabled, just pass through
        if (!filter_enabled_) {
            cloud_publisher_->publish(*msg);
            return;
        }

        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Try to convert with intensity, fall back to XYZ if it fails
        try {
            pcl::fromROSMsg(*msg, *cloud_in);
        } catch (...) {
            // If XYZI conversion fails, try XYZ and add zero intensity
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud_xyz);
            cloud_in->resize(cloud_xyz->size());
            for (size_t i = 0; i < cloud_xyz->size(); ++i) {
                cloud_in->points[i].x = cloud_xyz->points[i].x;
                cloud_in->points[i].y = cloud_xyz->points[i].y;
                cloud_in->points[i].z = cloud_xyz->points[i].z;
                cloud_in->points[i].intensity = 0;
            }
        }

        // Get transform from pcd's frame to vehicle frame if available
        geometry_msgs::msg::TransformStamped T_v_pcd;
        bool use_transform = false;
        
        try {
            // Check if the input frame is different from vehicle frame
            if (msg->header.frame_id != vehicle_frame_) {
                T_v_pcd = tf_buffer_->lookupTransform(
                    vehicle_frame_, msg->header.frame_id,
                    tf2::TimePointZero, std::chrono::milliseconds(100));
                use_transform = true;
            }
        } catch (tf2::TransformException &ex) {
            // If transform not available, filter in the original frame
            RCLCPP_DEBUG(this->get_logger(), "Transform not available, filtering in original frame: %s", ex.what());
        }

        geometry_msgs::msg::TransformStamped T_m_pcd;
        try {
            T_m_pcd = tf_buffer_->lookupTransform(
                map_frame_, msg->header.frame_id,
                tf2::TimePointZero, std::chrono::milliseconds(100));
        } catch (tf2::TransformException &ex) {
            static int tf_warn_count = 0;
            if (++tf_warn_count % 30 == 0) {
                RCLCPP_WARN(this->get_logger(),
                    "No transform from '%s' to '%s', skipping cloud (%d dropped): %s",
                    msg->header.frame_id.c_str(), map_frame_.c_str(),
                    tf_warn_count, ex.what());
            }
            return;
        }

        // Filter points
        cloud_filtered->reserve(cloud_in->size());
        cloud_filtered->header = cloud_in->header;
        
        for (const auto& point : cloud_in->points) {
            double px = point.x;
            double py = point.y;
            double pz = point.z;
            
            // Transform to vehicle frame if needed
            if (use_transform) {
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header = msg->header;
                point_in.point.x = px;
                point_in.point.y = py;
                point_in.point.z = pz;
                
                tf2::doTransform(point_in, point_out, T_v_pcd);
                px = point_out.point.x;
                py = point_out.point.y;
                pz = point_out.point.z;
            }
            
            // Check if point is outside the filter box (keep points outside the box)
            if (px < box_min_x_ || px > box_max_x_ ||
                py < box_min_y_ || py > box_max_y_ ||
                pz < box_min_z_ || pz > box_max_z_) {
                cloud_filtered->points.push_back(point);
            }
        }
        
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = false;

        // Transform point cloud to map frame using PCL
        // Convert geometry_msgs::TransformStamped to Eigen::Affine3f
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        
        // Set translation
        transform.translation() << T_m_pcd.transform.translation.x,
                                   T_m_pcd.transform.translation.y,
                                   T_m_pcd.transform.translation.z;
        
        // Set rotation from quaternion
        Eigen::Quaternionf q(
            T_m_pcd.transform.rotation.w,
            T_m_pcd.transform.rotation.x,
            T_m_pcd.transform.rotation.y,
            T_m_pcd.transform.rotation.z);
        transform.rotate(q);
        
        // Apply transformation using PCL
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, transform);

        // Convert back to ROS message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;
        output_msg.header.frame_id = map_frame_;
        
        // Log filtering statistics periodically
        static int callback_count = 0;
        if (++callback_count % 100 == 0) {
            int points_removed = cloud_in->size() - cloud_filtered->size();
            double removal_percentage = (double)points_removed / cloud_in->size() * 100.0;
            RCLCPP_DEBUG(this->get_logger(),
                "Filtered %d points (%.1f%%) from %zu total points",
                points_removed, removal_percentage, cloud_in->size());
        }

        // Publish box filter visualization marker
        auto box_marker = createBoxMarker(msg->header);
        marker_publisher_->publish(box_marker);

        cloud_publisher_->publish(output_msg);
    }

    void publishStateEstimation()
    {
        try {
            // Look up the latest transform from map to vehicle frame
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(
                    map_frame_, vehicle_frame_,
                    tf2::TimePointZero, std::chrono::milliseconds(50));

            // Create odometry message using the transform's timestamp
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = transform_stamped.header.stamp;  // Use transform's timestamp
            odom_msg.header.frame_id = map_frame_;
            odom_msg.child_frame_id = vehicle_frame_;

            // Set position from transform
            odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;

            // Set orientation from transform
            odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;

            // Publish the odometry message
            state_estimation_publisher_->publish(odom_msg);

            // Log state estimation periodically for debugging
            static int state_est_count = 0;
            if (++state_est_count % 1000 == 0) {
                RCLCPP_DEBUG(this->get_logger(),
                    "State estimation published: pos=[%.3f, %.3f, %.3f]",
                    odom_msg.pose.pose.position.x,
                    odom_msg.pose.pose.position.y,
                    odom_msg.pose.pose.position.z);
            }

        } catch (tf2::TransformException &ex) {
            // Log warning at a reduced rate to avoid spam
            static int error_count = 0;
            if (++error_count % 100 == 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Could not transform %s to %s: %s",
                    map_frame_.c_str(), vehicle_frame_.c_str(), ex.what());
            }
        }
    }

    visualization_msgs::msg::Marker createBoxMarker(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.header.frame_id = vehicle_frame_;
        marker.ns = "body_filter_box";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position at the center of the box
        marker.pose.position.x = (box_min_x_ + box_max_x_) / 2.0;
        marker.pose.position.y = (box_min_y_ + box_max_y_) / 2.0;
        marker.pose.position.z = (box_min_z_ + box_max_z_) / 2.0;

        // No rotation
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Scale is the size of the box
        marker.scale.x = box_max_x_ - box_min_x_;
        marker.scale.y = box_max_y_ - box_min_y_;
        marker.scale.z = box_max_z_ - box_min_z_;

        // Color: semi-transparent red to indicate filtered region
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;  // Semi-transparent

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent marker

        return marker;
    }

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_estimation_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr state_estimation_publish_timer_;
    
    // Frame names for static transforms
    std::string map_frame_;
    std::string init_frame_;
    std::string vehicle_frame_;
    std::string laser_imu_frame_;
    std::string laser_frame_;
    std::string init_odom_frame_;

    // T_v_i: Transform from IMU to vehicle (used for map→init_frame and imu→vehicle (inverse))
    double T_v_i_x_, T_v_i_y_, T_v_i_z_;
    double R_v_i_11_, R_v_i_12_, R_v_i_13_;
    double R_v_i_21_, R_v_i_22_, R_v_i_23_;
    double R_v_i_31_, R_v_i_32_, R_v_i_33_;
    
    // T_i_l: Transform from laser to IMU (laser extrinsic)
    double T_i_l_x_, T_i_l_y_, T_i_l_z_;
    double R_i_l_11_, R_i_l_12_, R_i_l_13_;
    double R_i_l_21_, R_i_l_22_, R_i_l_23_;
    double R_i_l_31_, R_i_l_32_, R_i_l_33_;
    
    // Box filter parameters
    bool filter_enabled_;
    double box_min_x_, box_max_x_;
    double box_min_y_, box_max_y_;
    double box_min_z_, box_max_z_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamInterface>());
    rclcpp::shutdown();
    return 0;
}
