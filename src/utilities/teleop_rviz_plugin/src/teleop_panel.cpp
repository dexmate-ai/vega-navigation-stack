#include "drive_widget.h"
#include "teleop_panel.h"
// #include 
#include <rclcpp/time.hpp>

namespace teleop_rviz_plugin
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz_common::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , mouse_pressed_( false )
  , mouse_pressed_sent_( false )
{
  node_ = rclcpp::Node::make_shared("teleop_panel_node");

  QVBoxLayout* layout = new QVBoxLayout;
  push_button_1_ = new QPushButton( "Resume Navigation to Goal", this );
  layout->addWidget( push_button_1_ );
  push_button_2_ = new QPushButton( "Clear Terrain Cloud", this );
  layout->addWidget( push_button_2_ );
  drive_widget_ = new DriveWidget;
  layout->addWidget( drive_widget_ );
  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  connect( push_button_1_, SIGNAL( pressed() ), this, SLOT( pressButton1() ));
  connect( push_button_2_, SIGNAL( pressed() ), this, SLOT( pressButton2() ));
  connect( drive_widget_, SIGNAL( outputVelocity( float, float, bool )), this, SLOT( setVel( float, float, bool )));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  output_timer->start( 100 );

  robot_cmd_publisher_ = node_->create_publisher<joy_command_mapper::msg::RobotCommand>("/robot_command", 5);

  drive_widget_->setEnabled( true );
}

void TeleopPanel::pressButton1()
{
  if (rclcpp::ok() && robot_cmd_publisher_->get_subscription_count() > 0)
  {
    auto robot_cmd = joy_command_mapper::msg::RobotCommand();
    robot_cmd.header.stamp = node_->now();
    robot_cmd.header.frame_id = "teleop_panel";
    robot_cmd.source = "rviz_teleop";

    // Set zero velocities
    robot_cmd.vx = 1.0;
    robot_cmd.vy = 1.0;
    robot_cmd.wz = 0.0;

    // Set modes
    robot_cmd.autonomy_mode = true;   // Enable autonomy
    robot_cmd.manual_mode = false;
    robot_cmd.obstacle_check_enabled = true;

    // Action triggers
    robot_cmd.clear_cloud = false;
    robot_cmd.emergency_stop = false;

    robot_cmd.deadman_switch = 1.0;

    robot_cmd_publisher_->publish(robot_cmd);
  }
}

void TeleopPanel::pressButton2()
{
  if (rclcpp::ok() && robot_cmd_publisher_->get_subscription_count() > 0)
  {
    auto robot_cmd = joy_command_mapper::msg::RobotCommand();
    robot_cmd.header.stamp = node_->now();
    robot_cmd.header.frame_id = "teleop_panel";
    robot_cmd.source = "rviz_teleop";

    robot_cmd.vx = 0.0;
    robot_cmd.vy = 0.0;
    robot_cmd.wz = 0.0;

    robot_cmd.autonomy_mode = false;
    robot_cmd.manual_mode = false;
    robot_cmd.obstacle_check_enabled = true;

    robot_cmd.clear_cloud = true;
    robot_cmd.emergency_stop = false;

    robot_cmd.deadman_switch = 0.0;

    robot_cmd_publisher_->publish(robot_cmd);
  }
}

void TeleopPanel::setVel(float lin, float ang, bool pre)
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
  mouse_pressed_ = pre;
}

void TeleopPanel::sendVel()
{
  if (rclcpp::ok() && (mouse_pressed_ || mouse_pressed_sent_) && robot_cmd_publisher_->get_subscription_count() > 0)
  {
    auto robot_cmd = joy_command_mapper::msg::RobotCommand();
    robot_cmd.header.stamp = node_->now();
    robot_cmd.header.frame_id = "teleop_panel";
    robot_cmd.source = "rviz_teleop";
    
    // Set velocities from drive widget
    robot_cmd.vx = linear_velocity_;
    robot_cmd.vy = angular_velocity_ * 0.5;  // Use angular as lateral for holonomic
    robot_cmd.wz = angular_velocity_;
    
    // Set modes (autonomy on, manual off)
    robot_cmd.autonomy_mode = false;
    robot_cmd.manual_mode = false;
    robot_cmd.obstacle_check_enabled = true;

    // No action triggers
    robot_cmd.clear_cloud = false;
    robot_cmd.emergency_stop = false;

    robot_cmd.deadman_switch = mouse_pressed_ ? 1.0 : 0.0;
    
    robot_cmd_publisher_->publish(robot_cmd);

    mouse_pressed_sent_ = mouse_pressed_;
  }
}

} // end namespace teleop_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(teleop_rviz_plugin::TeleopPanel, rviz_common::Panel)
