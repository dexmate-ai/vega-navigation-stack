#ifndef WAYPOINT_SEQUENCE_PANEL_HPP
#define WAYPOINT_SEQUENCE_PANEL_HPP

#ifndef Q_MOC_RUN
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/panel.hpp>
#endif

#include <QObject>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QSpinBox>
#include <QGridLayout>
#include <QMessageBox>
#include <QInputDialog>
#include <QTimer>

#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace rviz_common
{
class DisplayContext;
}

namespace waypoint_sequence_rviz_plugin
{

class WaypointSequencePanel : public rviz_common::Panel
{
  Q_OBJECT
  
public:
  WaypointSequencePanel(QWidget* parent = nullptr);
  
  virtual void onInitialize() override;

private Q_SLOTS:
  void onNewSequenceButtonClicked();
  void onAddCurrentPositionButtonClicked();
  void onManualInputButtonClicked();
  void onFinishSequenceButtonClicked();
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onPauseButtonClicked();
  void onResumeButtonClicked();
  void onResetButtonClicked();
  void onNextButtonClicked();
  void onClearButtonClicked();

private:
  void setupUI();
  void addWaypoint(double x, double y, double z);
  void clearWaypointList();
  void updateWaypointDisplay();
  void sendControlCommand(const std::string& command);
  void sendWaypointSequence();
  void onStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void onOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  // UI Elements
  QVBoxLayout* main_layout_;
  QGroupBox* sequence_group_;
  QGroupBox* control_group_;
  QGroupBox* status_group_;
  
  QPushButton* new_sequence_button_;
  QPushButton* add_current_position_button_;
  QPushButton* manual_input_button_;
  QPushButton* finish_sequence_button_;
  QPushButton* clear_button_;
  
  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* pause_button_;
  QPushButton* resume_button_;
  QPushButton* reset_button_;
  QPushButton* next_button_;
  
  QTextEdit* waypoint_display_;
  QLabel* status_label_;
  
  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sequence_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Timer for spinning the node
  QTimer* spin_timer_;
  
  // State
  std::vector<std::array<double, 3>> waypoints_;
  bool sequence_recording_;
  std::string current_status_;
  
  // Robot state
  geometry_msgs::msg::Point current_robot_position_;
  bool robot_position_available_;
};

} // namespace waypoint_sequence_rviz_plugin

#endif // WAYPOINT_SEQUENCE_PANEL_HPP