#include <waypoint_tool.hpp>

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace waypoint_rviz_plugin
{
WaypointTool::WaypointTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

WaypointTool::~WaypointTool() = default;

void WaypointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("Waypoint");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool::updateTopic()
{
  auto ros_context = context_->getRosNodeAbstraction().lock();
  if (!ros_context) {
    RCLCPP_WARN(rclcpp::get_logger("waypoint_tool"), "Unable to access RViz ROS context");
    return;
  }

  raw_node_ = ros_context->get_raw_node();
  if (!raw_node_) {
    RCLCPP_WARN(rclcpp::get_logger("waypoint_tool"), "Unable to access RViz raw node");
    return;
  }

  sub_ = raw_node_->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5, std::bind(&WaypointTool::odomHandler, this, std::placeholders::_1));

  pub_ = raw_node_->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", qos_profile_);
  robot_cmd_pub_ = raw_node_->create_publisher<joy_command_mapper::msg::RobotCommand>("/robot_command", qos_profile_);
  clock_ = raw_node_->get_clock();
}

void WaypointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  (void)theta;

  if (!clock_) {
    RCLCPP_WARN(rclcpp::get_logger("waypoint_tool"), "Clock unavailable; cannot publish waypoint command");
    return;
  }

  auto now = clock_->now();

  if (robot_cmd_pub_) {
    auto robot_cmd = joy_command_mapper::msg::RobotCommand();
    robot_cmd.header.stamp = now;
    robot_cmd.header.frame_id = "waypoint_tool";
    robot_cmd.source = "waypoint_tool";

    robot_cmd.vx = 1.0f;
    robot_cmd.vy = 0.0f;
    robot_cmd.wz = 0.0f;

    robot_cmd.autonomy_mode = true;
    robot_cmd.manual_mode = false;
    robot_cmd.obstacle_check_enabled = true;

    robot_cmd.clear_cloud = false;
    robot_cmd.emergency_stop = false;

    robot_cmd.deadman_switch = 1.0f;
    robot_cmd_pub_->publish(robot_cmd);
  }

  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = now;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;

  pub_->publish(waypoint);
  usleep(10000);
  pub_->publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::WaypointTool, rviz_common::Tool)
