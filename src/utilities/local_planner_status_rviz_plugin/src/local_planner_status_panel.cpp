#include "local_planner_status_panel.hpp"

#include <sstream>
#include <iomanip>

namespace local_planner_status_rviz_plugin
{

LocalPlannerStatusPanel::LocalPlannerStatusPanel(QWidget* parent)
: rviz_common::Panel(parent)
, goal_received_(false)
, current_nav_status_(0)
, last_goal_status_(nullptr)
{
  setupUI();
}

void LocalPlannerStatusPanel::onInitialize()
{
  // Create our own node
  node_ = rclcpp::Node::make_shared("local_planner_status_panel_node");

  // Create subscriber for goal updates
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/way_point", 10,
    std::bind(&LocalPlannerStatusPanel::onGoalCallback, this, std::placeholders::_1));

  // Create subscriber for nav status updates
  nav_status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/nav_status", 10,
    std::bind(&LocalPlannerStatusPanel::onNavStatusCallback, this, std::placeholders::_1));

  // Create subscriber for detailed goal status updates
  goal_status_sub_ = node_->create_subscription<autonomy_msgs::msg::GoalStatus>(
    "/goal_status", 10,
    std::bind(&LocalPlannerStatusPanel::onGoalStatusCallback, this, std::placeholders::_1));

  // Set up timer to spin the node
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
  });
  spin_timer_->start(100); // Spin every 100ms

  // Set up timer to update the display
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &LocalPlannerStatusPanel::updateDisplay);
  update_timer_->start(200); // Update display every 200ms
}

void LocalPlannerStatusPanel::setupUI()
{
  main_layout_ = new QVBoxLayout();
  setLayout(main_layout_);

  // Status Group
  status_group_ = new QGroupBox("Local Planner Status Monitor");
  QVBoxLayout* status_layout = new QVBoxLayout();

  // Goal received indicator
  goal_received_label_ = new QLabel("Goal Received: No");
  goal_received_label_->setStyleSheet("QLabel { font-weight: bold; }");
  status_layout->addWidget(goal_received_label_);

  // Goal position
  goal_position_label_ = new QLabel("Goal Position: N/A");
  status_layout->addWidget(goal_position_label_);

  // Time since last goal
  time_since_goal_label_ = new QLabel("Time Since Goal: N/A");
  status_layout->addWidget(time_since_goal_label_);

  // Navigation status
  nav_status_label_ = new QLabel("Nav Status: Unknown");
  nav_status_label_->setStyleSheet("QLabel { font-weight: bold; }");
  status_layout->addWidget(nav_status_label_);

  // Add separator
  QLabel* separator = new QLabel("--- Goal Status Details ---");
  separator->setStyleSheet("QLabel { font-weight: bold; color: gray; margin-top: 10px; }");
  status_layout->addWidget(separator);

  // Goal distance
  goal_distance_label_ = new QLabel("Goal Distance: N/A");
  status_layout->addWidget(goal_distance_label_);

  // Goal clear range
  goal_clear_range_label_ = new QLabel("Goal Clear Range: N/A");
  status_layout->addWidget(goal_clear_range_label_);

  // Orientation aligned
  orientation_aligned_label_ = new QLabel("Orientation Aligned: N/A");
  status_layout->addWidget(orientation_aligned_label_);

  // Yaw error
  yaw_error_label_ = new QLabel("Yaw Error: N/A");
  status_layout->addWidget(yaw_error_label_);

  status_group_->setLayout(status_layout);
  main_layout_->addWidget(status_group_);

  // Add stretch to push everything to the top
  main_layout_->addStretch();
}

void LocalPlannerStatusPanel::onGoalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_received_ = true;
  last_goal_time_ = node_->now();
  last_goal_position_ = msg->point;
}

void LocalPlannerStatusPanel::onNavStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  current_nav_status_ = msg->data;
}

void LocalPlannerStatusPanel::onGoalStatusCallback(const autonomy_msgs::msg::GoalStatus::SharedPtr msg)
{
  last_goal_status_ = msg;
}

void LocalPlannerStatusPanel::updateDisplay()
{
  // Update goal received status
  if (goal_received_) {
    goal_received_label_->setText("Goal Received: Yes");
    goal_received_label_->setStyleSheet("QLabel { font-weight: bold; color: green; }");

    // Update goal position
    std::stringstream pos_ss;
    pos_ss << "Goal Position: ("
           << std::fixed << std::setprecision(2)
           << last_goal_position_.x << ", "
           << last_goal_position_.y << ", "
           << last_goal_position_.z << ")";
    goal_position_label_->setText(QString::fromStdString(pos_ss.str()));

    // Update time since goal
    if (rclcpp::ok() && node_) {
      double elapsed_seconds = (node_->now() - last_goal_time_).seconds();
      std::string time_str = formatDuration(elapsed_seconds);
      time_since_goal_label_->setText(QString::fromStdString("Time Since Goal: " + time_str));
    }
  } else {
    goal_received_label_->setText("Goal Received: No");
    goal_received_label_->setStyleSheet("QLabel { font-weight: bold; color: gray; }");
    goal_position_label_->setText("Goal Position: N/A");
    time_since_goal_label_->setText("Time Since Goal: N/A");
  }

  // Update navigation status
  std::string status_text = decodeNavStatus(current_nav_status_);
  nav_status_label_->setText(QString::fromStdString("Nav Status: " + status_text));

  // Set color based on status
  if (current_nav_status_ & NAV_STATUS_GOAL_REACHED) {
    nav_status_label_->setStyleSheet("QLabel { font-weight: bold; color: blue; }");
  } else if (current_nav_status_ & NAV_STATUS_GOAL_STUCK) {
    nav_status_label_->setStyleSheet("QLabel { font-weight: bold; color: red; }");
  } else if (current_nav_status_ & NAV_STATUS_ACTIVE) {
    nav_status_label_->setStyleSheet("QLabel { font-weight: bold; color: green; }");
  } else if (current_nav_status_ & NAV_STATUS_MANUAL) {
    nav_status_label_->setStyleSheet("QLabel { font-weight: bold; color: orange; }");
  } else {
    nav_status_label_->setStyleSheet("QLabel { font-weight: bold; color: gray; }");
  }

  // Update detailed goal status information
  if (last_goal_status_) {
    // Goal distance
    std::stringstream dist_ss;
    dist_ss << "Goal Distance: " << std::fixed << std::setprecision(3)
            << last_goal_status_->goal_distance << " m";
    goal_distance_label_->setText(QString::fromStdString(dist_ss.str()));

    // Color code based on distance vs threshold
    if (last_goal_status_->goal_reached) {
      goal_distance_label_->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
    } else if (last_goal_status_->goal_distance <= last_goal_status_->goal_clear_range * 1.5) {
      goal_distance_label_->setStyleSheet("QLabel { color: orange; }");
    } else {
      goal_distance_label_->setStyleSheet("QLabel { color: black; }");
    }

    // Goal clear range
    std::stringstream range_ss;
    range_ss << "Goal Clear Range: " << std::fixed << std::setprecision(3)
             << last_goal_status_->goal_clear_range << " m";
    goal_clear_range_label_->setText(QString::fromStdString(range_ss.str()));

    // Orientation aligned
    std::string aligned_str = last_goal_status_->orientation_aligned ? "Yes" : "No";
    orientation_aligned_label_->setText(QString::fromStdString("Orientation Aligned: " + aligned_str));
    if (last_goal_status_->orientation_aligned) {
      orientation_aligned_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    } else {
      orientation_aligned_label_->setStyleSheet("QLabel { color: red; }");
    }

    // Yaw error
    std::stringstream yaw_ss;
    double yaw_error_deg = last_goal_status_->yaw_error * 180.0 / M_PI;
    yaw_ss << "Yaw Error: " << std::fixed << std::setprecision(2)
           << yaw_error_deg << " deg ("
           << std::fixed << std::setprecision(3)
           << last_goal_status_->yaw_error << " rad)";
    yaw_error_label_->setText(QString::fromStdString(yaw_ss.str()));

    if (std::abs(yaw_error_deg) < 5.0) {
      yaw_error_label_->setStyleSheet("QLabel { color: green; }");
    } else if (std::abs(yaw_error_deg) < 10.0) {
      yaw_error_label_->setStyleSheet("QLabel { color: orange; }");
    } else {
      yaw_error_label_->setStyleSheet("QLabel { color: red; }");
    }
  } else {
    goal_distance_label_->setText("Goal Distance: N/A");
    goal_distance_label_->setStyleSheet("QLabel { color: gray; }");
    goal_clear_range_label_->setText("Goal Clear Range: N/A");
    orientation_aligned_label_->setText("Orientation Aligned: N/A");
    orientation_aligned_label_->setStyleSheet("QLabel { color: gray; }");
    yaw_error_label_->setText("Yaw Error: N/A");
    yaw_error_label_->setStyleSheet("QLabel { color: gray; }");
  }
}

std::string LocalPlannerStatusPanel::formatDuration(double seconds)
{
  std::stringstream ss;

  if (seconds < 60.0) {
    ss << std::fixed << std::setprecision(1) << seconds << "s";
  } else if (seconds < 3600.0) {
    int minutes = static_cast<int>(seconds / 60.0);
    int secs = static_cast<int>(seconds) % 60;
    ss << minutes << "m " << secs << "s";
  } else {
    int hours = static_cast<int>(seconds / 3600.0);
    int minutes = static_cast<int>(seconds / 60.0) % 60;
    ss << hours << "h " << minutes << "m";
  }

  return ss.str();
}

std::string LocalPlannerStatusPanel::decodeNavStatus(int32_t status)
{
  if (status == 0) {
    return "Idle (0)";
  }

  std::vector<std::string> flags;

  if (status & NAV_STATUS_ACTIVE) {
    flags.push_back("Active");
  }
  if (status & NAV_STATUS_MANUAL) {
    flags.push_back("Manual");
  }
  if (status & NAV_STATUS_GOAL_REACHED) {
    flags.push_back("Goal Reached");
  }
  if (status & NAV_STATUS_GOAL_STUCK) {
    flags.push_back("Goal Stuck");
  }

  std::stringstream ss;
  for (size_t i = 0; i < flags.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << flags[i];
  }

  ss << " (" << status << ")";
  return ss.str();
}

} // namespace local_planner_status_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(local_planner_status_rviz_plugin::LocalPlannerStatusPanel, rviz_common::Panel)
