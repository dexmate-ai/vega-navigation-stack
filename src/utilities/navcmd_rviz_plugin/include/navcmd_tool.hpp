#ifndef NAVCMD_TOOL_H
#define NAVCMD_TOOL_H


#include <QObject>

#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <autonomy_msgs/msg/navigate_cmd.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/tool.hpp>

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class BoolProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace navcmd_rviz_plugin
{
class NavCmdTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT
public:
  NavCmdTool();

  ~NavCmdTool() override;

  virtual void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<autonomy_msgs::msg::NavigateCmd>::SharedPtr nav_cmd_pub_;
  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;
  rviz_common::properties::BoolProperty * allow_reverse_property_;
  rviz_common::properties::BoolProperty * track_yaw_property_;

  rclcpp::QoS qos_profile_;
};
}


#endif  // NAVCMD_RVIZ_PLUGIN_NAVCMD_TOOL_H
