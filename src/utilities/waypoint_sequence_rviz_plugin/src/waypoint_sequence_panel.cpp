#include "waypoint_sequence_panel.hpp"

#include <sstream>
#include <iomanip>

namespace waypoint_sequence_rviz_plugin
{

WaypointSequencePanel::WaypointSequencePanel(QWidget* parent)
: rviz_common::Panel(parent)
, sequence_recording_(false)
, current_status_("UNKNOWN")
, robot_position_available_(false)
{
  setupUI();
}

void WaypointSequencePanel::onInitialize()
{
  // Create our own node
  node_ = rclcpp::Node::make_shared("waypoint_sequence_panel_node");
  
  // Create publishers
  sequence_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/new_waypoint_sequence", 10);
  control_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/waypoint_sequence_control", 10);
  
  // Create subscriber for status updates
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/waypoint_sequence_status", 10,
    std::bind(&WaypointSequencePanel::onStatusCallback, this, std::placeholders::_1));
  
  // Create subscriber for robot odometry
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 10,
    std::bind(&WaypointSequencePanel::onOdometryCallback, this, std::placeholders::_1));
  
  // Set up timer to spin the node
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
  });
  spin_timer_->start(100); // Spin every 100ms
}

void WaypointSequencePanel::setupUI()
{
  main_layout_ = new QVBoxLayout();
  setLayout(main_layout_);
  
  // Sequence Creation Group
  sequence_group_ = new QGroupBox("Waypoint Sequence Creation");
  QVBoxLayout* sequence_layout = new QVBoxLayout();
  
  new_sequence_button_ = new QPushButton("New Sequence");
  add_current_position_button_ = new QPushButton("Add Current Position");
  manual_input_button_ = new QPushButton("Manual Input Waypoint");
  finish_sequence_button_ = new QPushButton("Finish Sequence");
  clear_button_ = new QPushButton("Clear Waypoints");
  
  add_current_position_button_->setEnabled(false);
  manual_input_button_->setEnabled(false);
  finish_sequence_button_->setEnabled(false);
  
  sequence_layout->addWidget(new_sequence_button_);
  sequence_layout->addWidget(add_current_position_button_);
  sequence_layout->addWidget(manual_input_button_);
  sequence_layout->addWidget(finish_sequence_button_);
  sequence_layout->addWidget(clear_button_);
  
  waypoint_display_ = new QTextEdit();
  waypoint_display_->setMaximumHeight(100);
  waypoint_display_->setReadOnly(true);
  waypoint_display_->setText("No waypoints added");
  sequence_layout->addWidget(waypoint_display_);
  
  sequence_group_->setLayout(sequence_layout);
  main_layout_->addWidget(sequence_group_);
  
  // Control Group
  control_group_ = new QGroupBox("Sequence Control");
  QGridLayout* control_layout = new QGridLayout();
  
  start_button_ = new QPushButton("Start");
  stop_button_ = new QPushButton("Stop");
  pause_button_ = new QPushButton("Pause");
  resume_button_ = new QPushButton("Resume");
  reset_button_ = new QPushButton("Reset");
  next_button_ = new QPushButton("Next Waypoint");
  
  control_layout->addWidget(start_button_, 0, 0);
  control_layout->addWidget(stop_button_, 0, 1);
  control_layout->addWidget(pause_button_, 1, 0);
  control_layout->addWidget(resume_button_, 1, 1);
  control_layout->addWidget(reset_button_, 2, 0);
  control_layout->addWidget(next_button_, 2, 1);
  
  control_group_->setLayout(control_layout);
  main_layout_->addWidget(control_group_);
  
  // Status Group
  status_group_ = new QGroupBox("Status");
  QVBoxLayout* status_layout = new QVBoxLayout();
  
  status_label_ = new QLabel("Status: UNKNOWN");
  status_layout->addWidget(status_label_);
  
  status_group_->setLayout(status_layout);
  main_layout_->addWidget(status_group_);
  
  // Connect signals
  connect(new_sequence_button_, SIGNAL(clicked()), this, SLOT(onNewSequenceButtonClicked()));
  connect(add_current_position_button_, SIGNAL(clicked()), this, SLOT(onAddCurrentPositionButtonClicked()));
  connect(manual_input_button_, SIGNAL(clicked()), this, SLOT(onManualInputButtonClicked()));
  connect(finish_sequence_button_, SIGNAL(clicked()), this, SLOT(onFinishSequenceButtonClicked()));
  connect(clear_button_, SIGNAL(clicked()), this, SLOT(onClearButtonClicked()));
  
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
  connect(resume_button_, SIGNAL(clicked()), this, SLOT(onResumeButtonClicked()));
  connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetButtonClicked()));
  connect(next_button_, SIGNAL(clicked()), this, SLOT(onNextButtonClicked()));
}

void WaypointSequencePanel::onNewSequenceButtonClicked()
{
  clearWaypointList();
  sequence_recording_ = true;
  
  new_sequence_button_->setEnabled(false);
  add_current_position_button_->setEnabled(true);
  manual_input_button_->setEnabled(true);
  finish_sequence_button_->setEnabled(true);
  
  QMessageBox::information(this, "New Sequence", 
    "Use 'Add Current Position' to add robot's current position,\n"
    "or 'Manual Input Waypoint' to specify coordinates manually.\n"
    "Click 'Finish Sequence' when done.");
}

void WaypointSequencePanel::onAddCurrentPositionButtonClicked()
{
  if (!robot_position_available_) {
    QMessageBox::warning(this, "No Robot Position", 
      "Robot position is not available. Make sure the robot is publishing to /state_estimation topic.");
    return;
  }
  
  addWaypoint(current_robot_position_.x, current_robot_position_.y, current_robot_position_.z);
}

void WaypointSequencePanel::onManualInputButtonClicked()
{
  bool ok_x, ok_y, ok_z;
  double x = QInputDialog::getDouble(this, "Add Waypoint", "X coordinate:", 0.0, -1000.0, 1000.0, 2, &ok_x);
  if (!ok_x) return;
  
  double y = QInputDialog::getDouble(this, "Add Waypoint", "Y coordinate:", 0.0, -1000.0, 1000.0, 2, &ok_y);
  if (!ok_y) return;
  
  double z = QInputDialog::getDouble(this, "Add Waypoint", "Z coordinate:", 0.0, -100.0, 100.0, 2, &ok_z);
  if (!ok_z) return;
  
  addWaypoint(x, y, z);
}

void WaypointSequencePanel::onFinishSequenceButtonClicked()
{
  if (waypoints_.empty()) {
    QMessageBox::warning(this, "Empty Sequence", "Please add at least one waypoint before finishing.");
    return;
  }
  
  sequence_recording_ = false;
  sendWaypointSequence();
  
  new_sequence_button_->setEnabled(true);
  add_current_position_button_->setEnabled(false);
  manual_input_button_->setEnabled(false);
  finish_sequence_button_->setEnabled(false);
  
  QMessageBox::information(this, "Sequence Complete", 
    QString("Waypoint sequence with %1 points has been sent to the manager.").arg(waypoints_.size()));
}

void WaypointSequencePanel::onStartButtonClicked()
{
  sendControlCommand("START");
}

void WaypointSequencePanel::onStopButtonClicked()
{
  sendControlCommand("STOP");
}

void WaypointSequencePanel::onPauseButtonClicked()
{
  sendControlCommand("PAUSE");
}

void WaypointSequencePanel::onResumeButtonClicked()
{
  sendControlCommand("RESUME");
}

void WaypointSequencePanel::onResetButtonClicked()
{
  sendControlCommand("RESET");
}

void WaypointSequencePanel::onNextButtonClicked()
{
  sendControlCommand("NEXT");
}

void WaypointSequencePanel::onClearButtonClicked()
{
  clearWaypointList();
}

void WaypointSequencePanel::addWaypoint(double x, double y, double z)
{
  waypoints_.push_back({x, y, z});
  updateWaypointDisplay();
}

void WaypointSequencePanel::clearWaypointList()
{
  waypoints_.clear();
  updateWaypointDisplay();
}

void WaypointSequencePanel::updateWaypointDisplay()
{
  if (waypoints_.empty()) {
    waypoint_display_->setText("No waypoints added");
    return;
  }
  
  std::stringstream ss;
  ss << "Waypoints (" << waypoints_.size() << "):\n";
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    ss << i + 1 << ": (" 
       << std::fixed << std::setprecision(2) 
       << waypoints_[i][0] << ", " 
       << waypoints_[i][1] << ", " 
       << waypoints_[i][2] << ")\n";
  }
  
  waypoint_display_->setText(QString::fromStdString(ss.str()));
}

void WaypointSequencePanel::sendControlCommand(const std::string& command)
{
  if (!control_pub_) return;
  
  std_msgs::msg::String msg;
  msg.data = command;
  control_pub_->publish(msg);
}

void WaypointSequencePanel::sendWaypointSequence()
{
  if (!sequence_pub_) return;
  
  std::stringstream json_ss;
  json_ss << "{\"waypoints\":[";
  
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    if (i > 0) json_ss << ",";
    json_ss << "{\"x\":" << waypoints_[i][0] 
            << ",\"y\":" << waypoints_[i][1] 
            << ",\"z\":" << waypoints_[i][2] << "}";
  }
  
  json_ss << "]}";
  
  std_msgs::msg::String msg;
  msg.data = json_ss.str();
  sequence_pub_->publish(msg);
}

void WaypointSequencePanel::onStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Parse status: "STATUS|total_waypoints|current_index|sequence_id"
  std::string status_data = msg->data;
  std::stringstream ss(status_data);
  std::string status, total_wp, current_wp, seq_id;
  
  if (std::getline(ss, status, '|') &&
      std::getline(ss, total_wp, '|') &&
      std::getline(ss, current_wp, '|') &&
      std::getline(ss, seq_id, '|')) {
    
    current_status_ = status;
    QString display_text = QString("Status: %1\nWaypoints: %2/%3\nSequence ID: %4")
                          .arg(QString::fromStdString(status))
                          .arg(QString::fromStdString(current_wp))
                          .arg(QString::fromStdString(total_wp))
                          .arg(QString::fromStdString(seq_id));
    
    status_label_->setText(display_text);
  }
}

void WaypointSequencePanel::onOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Store the robot's current position
  current_robot_position_ = msg->pose.pose.position;
  robot_position_available_ = true;
}

} // namespace waypoint_sequence_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_sequence_rviz_plugin::WaypointSequencePanel, rviz_common::Panel)