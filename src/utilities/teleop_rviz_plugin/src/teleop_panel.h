#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/panel.hpp>
#endif

#include <stdio.h>

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QCheckBox>
#include <QFileDialog>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <joy_command_mapper/msg/robot_command.hpp>

class QLineEdit;

namespace teleop_rviz_plugin
{

class DriveWidget;

class TeleopPanel: public rviz_common::Panel
{
Q_OBJECT
public:
  TeleopPanel( QWidget* parent = 0 );

public Q_SLOTS:
  void setVel( float linear_velocity_, float angular_velocity_, bool mouse_pressed_ );

protected Q_SLOTS:
  void pressButton1();
  void pressButton2();
  void sendVel();

protected:
  DriveWidget* drive_widget_;

  // ROS2 uses smart pointers for publishers and subscribers
  rclcpp::Publisher<joy_command_mapper::msg::RobotCommand>::SharedPtr robot_cmd_publisher_;

  // Use shared pointer for the node
  rclcpp::Node::SharedPtr node_;

  QPushButton *push_button_1_;
  QPushButton *push_button_2_;

  float linear_velocity_;
  float angular_velocity_;
  bool mouse_pressed_;
  bool mouse_pressed_sent_;
};

}

#endif // TELEOP_PANEL_H
