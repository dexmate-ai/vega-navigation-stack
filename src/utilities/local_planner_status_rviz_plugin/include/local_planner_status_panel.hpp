#ifndef LOCAL_PLANNER_STATUS_PANEL_HPP
#define LOCAL_PLANNER_STATUS_PANEL_HPP

#ifndef Q_MOC_RUN
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/panel.hpp>
#endif

#include <QObject>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>

#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <autonomy_msgs/msg/goal_status.hpp>
#include <chrono>

namespace rviz_common
{
class DisplayContext;
}

namespace local_planner_status_rviz_plugin
{

class LocalPlannerStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  LocalPlannerStatusPanel(QWidget* parent = nullptr);

  virtual void onInitialize() override;

private Q_SLOTS:
  void updateDisplay();

private:
  void setupUI();
  void onGoalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onNavStatusCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void onGoalStatusCallback(const autonomy_msgs::msg::GoalStatus::SharedPtr msg);
  std::string formatDuration(double seconds);
  std::string decodeNavStatus(int32_t status);

  // UI Elements
  QVBoxLayout* main_layout_;
  QGroupBox* status_group_;

  QLabel* goal_received_label_;
  QLabel* time_since_goal_label_;
  QLabel* nav_status_label_;
  QLabel* goal_position_label_;
  QLabel* goal_distance_label_;
  QLabel* goal_clear_range_label_;
  QLabel* orientation_aligned_label_;
  QLabel* yaw_error_label_;

  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<autonomy_msgs::msg::GoalStatus>::SharedPtr goal_status_sub_;

  // Timer for spinning the node and updating display
  QTimer* spin_timer_;
  QTimer* update_timer_;

  // State
  bool goal_received_;
  rclcpp::Time last_goal_time_;
  geometry_msgs::msg::Point last_goal_position_;
  int32_t current_nav_status_;
  autonomy_msgs::msg::GoalStatus::SharedPtr last_goal_status_;

  // Nav status bit flags
  static constexpr int32_t NAV_STATUS_ACTIVE = 1 << 0;
  static constexpr int32_t NAV_STATUS_GOAL_REACHED = 1 << 2;
  static constexpr int32_t NAV_STATUS_GOAL_STUCK = 1 << 3;
  static constexpr int32_t NAV_STATUS_MANUAL = 1 << 4;
};

} // namespace local_planner_status_rviz_plugin

#endif // LOCAL_PLANNER_STATUS_PANEL_HPP
