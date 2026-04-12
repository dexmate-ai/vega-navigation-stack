#include <navcmd_tool.hpp>

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace navcmd_rviz_plugin
{
NavCmdTool::NavCmdTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'n';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "nav_cmd",
    "The topic on which to publish navigation commands.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);

  allow_reverse_property_ = new rviz_common::properties::BoolProperty(
    "Allow Reverse", false,
    "Allow the robot to move in reverse during navigation.",
    getPropertyContainer(), nullptr, this);

  track_yaw_property_ = new rviz_common::properties::BoolProperty(
    "Track Yaw", true,
    "Track goal yaw orientation (if false, only position matters).",
    getPropertyContainer(), nullptr, this);
}

NavCmdTool::~NavCmdTool() = default;

void NavCmdTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("NavCmd");
  updateTopic();
}

void NavCmdTool::updateTopic()
{
  auto ros_context = context_->getRosNodeAbstraction().lock();
  if (!ros_context) {
    RCLCPP_WARN(rclcpp::get_logger("navcmd_tool"), "Unable to access RViz ROS context");
    return;
  }

  raw_node_ = ros_context->get_raw_node();
  if (!raw_node_) {
    RCLCPP_WARN(rclcpp::get_logger("navcmd_tool"), "Unable to access RViz raw node");
    return;
  }

  nav_cmd_pub_ = raw_node_->create_publisher<autonomy_msgs::msg::NavigateCmd>(
    "/nav_cmd", qos_profile_);
  clock_ = raw_node_->get_clock();
}

void NavCmdTool::onPoseSet(double x, double y, double theta)
{
  if (!clock_) {
    RCLCPP_WARN(rclcpp::get_logger("navcmd_tool"),
      "Clock unavailable; cannot publish navigation command");
    return;
  }

  auto now = clock_->now();

  if (nav_cmd_pub_) {
    auto nav_cmd = autonomy_msgs::msg::NavigateCmd();

    // Set goal pose
    nav_cmd.goal_pose.header.stamp = now;
    nav_cmd.goal_pose.header.frame_id = "map";
    nav_cmd.goal_pose.pose.position.x = x;
    nav_cmd.goal_pose.pose.position.y = y;
    nav_cmd.goal_pose.pose.position.z = 0.0;

    // Convert theta to quaternion
    nav_cmd.goal_pose.pose.orientation.x = 0.0;
    nav_cmd.goal_pose.pose.orientation.y = 0.0;
    nav_cmd.goal_pose.pose.orientation.z = std::sin(theta / 2.0);
    nav_cmd.goal_pose.pose.orientation.w = std::cos(theta / 2.0);

    // Set autonomy and reverse settings
    nav_cmd.is_autonomy = true;
    nav_cmd.allow_reverse = allow_reverse_property_->getBool();
    nav_cmd.track_yaw = track_yaw_property_->getBool();
    nav_cmd.check_obstacle = true;

    nav_cmd_pub_->publish(nav_cmd);

    RCLCPP_INFO(rclcpp::get_logger("navcmd_tool"),
      "Published nav_cmd to (%.2f, %.2f, %.2f rad) with autonomy=true, allow_reverse=%s, track_yaw=%s",
      x, y, theta, 
      nav_cmd.allow_reverse ? "true" : "false",
      nav_cmd.track_yaw ? "true" : "false");
  }
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navcmd_rviz_plugin::NavCmdTool, rviz_common::Tool)
