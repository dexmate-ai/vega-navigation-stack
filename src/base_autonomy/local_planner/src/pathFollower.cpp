#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <autonomy_msgs/msg/navigate_cmd.hpp>
#include <joy_command_mapper/msg/robot_command.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;
using std::placeholders::_1;

const double PI = 3.1415926;

// PathFollowerNode owns the pure pursuit controller and ROS interfaces, eliminating the
// reliance on global state while keeping the control loop easy to follow.
class PathFollowerNode
{
public:
  explicit PathFollowerNode(const rclcpp::Node::SharedPtr &node)
  : node_(node)
  {
    declareParameters();
    loadParameters();
    setupPublishers();
    setupSubscriptions();
    initializeAutonomyState();
    cmdVel_.header.frame_id = "vehicle";
    controlTimer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&PathFollowerNode::processControlStep, this));
  }

private:
  rclcpp::Node::SharedPtr node_;

  // Parameters
  double sensorOffsetX = 0;
  double sensorOffsetY = 0;
  int pubSkipNum = 1;
  bool twoWayDrive = true;
  double lookAheadDis = 0.5;
  double yawRateGain = 7.5;
  double stopYawRateGain = 7.5;
  double maxYawRate = 45.0;
  double maxYawAccel = 180.0;
  double maxSpeed = 1.0;
  double maxAccel = 1.0;
  double switchTimeThre = 1.0;
  double dirDiffThre = 0.1;
  double omniDirGoalThre = 1.0;
  double omniDirDiffThre = 1.5;
  double slowDwnDisThre = 1.0;
  bool useInclRateToSlow = false;
  double inclRateThre = 120.0;
  double slowRate1 = 0.25;
  double slowRate2 = 0.5;
  double slowTime1 = 2.0;
  double slowTime2 = 2.0;
  bool useInclToStop = false;
  double inclThre = 45.0;
  double stopTime = 5.0;
  bool noRotAtStop = false;
  bool autonomyMode = false;
  double autonomySpeed = 1.0;
  double joyToSpeedDelay = 2.0;
  double goalYawToleranceDeg = 5.0;
  double goalYawToleranceRad = goalYawToleranceDeg * PI / 180.0;
  double finalYawActivationDist = 0.3;
  double finalYawHysteresis = 1.5;
  double goalYaw = 0.0;
  bool goalYawValid = false;
  double yawRateFilterAlpha = 0.3;
  double velFilterAlpha = 0.3;
  double finalApproachDist = 0.3;  // Distance to switch to final approach mode
  double finalApproachSpeed = 0.2;  // Speed during final approach (m/s)
  double finalApproachGain = 1.5;  // P-gain for final approach velocity control

  // Final approach mode state: 0=NORMAL, 1=FINAL_XY_APPROACH, 2=FINAL_YAW_ADJUST
  int finalApproachMode = 0;
  // Actual goal position in world frame
  double goalX = 0.0;
  double goalY = 0.0;
  bool goalPositionValid = false;

  // Runtime state
  int pubSkipCount = 0;
  float joySpeed = 0;
  float joySpeedRaw = 0;
  float joyYaw = 0;
  float joyManualFwd = 0;
  float joyManualLeft = 0;
  float joyManualYaw = 0;
  int safetyStop = 0;
  int slowDown = 0;
  bool manualMode = false;

  float vehicleX = 0;
  float vehicleY = 0;
  float vehicleZ = 0;
  float vehicleRoll = 0;
  float vehiclePitch = 0;
  float vehicleYaw = 0;

  float vehicleXRec = 0;
  float vehicleYRec = 0;
  float vehicleZRec = 0;
  float vehicleRollRec = 0;
  float vehiclePitchRec = 0;
  float vehicleYawRec = 0;

  float vehicleYawRate = 0;
  float targetYawRate = 0;
  float filteredTargetYawRate = 0;
  float vehicleSpeed = 0;
  float filteredVelX = 0;
  float filteredVelY = 0;

  double odomTime = 0;
  double joyTime = 0;
  double slowInitTime = 0;
  double stopInitTime = 0;
  int pathPointID = 0;
  bool pathInit = false;
  bool navFwd = true;
  double switchTime = 0;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::TwistStamped cmdVel_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
  rclcpp::Subscription<joy_command_mapper::msg::RobotCommand>::SharedPtr subRobotCommand;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subStop;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subSlowDown;
  rclcpp::Subscription<autonomy_msgs::msg::NavigateCmd>::SharedPtr subNavigateCmd;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubSpeed;
  rclcpp::TimerBase::SharedPtr controlTimer_;

  void declareParameters()
  {
    node_->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
    node_->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
    node_->declare_parameter<int>("pubSkipNum", pubSkipNum);
    node_->declare_parameter<bool>("twoWayDrive", twoWayDrive);
    node_->declare_parameter<double>("lookAheadDis", lookAheadDis);
    node_->declare_parameter<double>("yawRateGain", yawRateGain);
    node_->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
    node_->declare_parameter<double>("maxYawRate", maxYawRate);
    node_->declare_parameter<double>("maxYawAccel", maxYawAccel);
    node_->declare_parameter<double>("maxSpeed", maxSpeed);
    node_->declare_parameter<double>("maxAccel", maxAccel);
    node_->declare_parameter<double>("switchTimeThre", switchTimeThre);
    node_->declare_parameter<double>("dirDiffThre", dirDiffThre);
    node_->declare_parameter<double>("omniDirGoalThre", omniDirGoalThre);
    node_->declare_parameter<double>("omniDirDiffThre", omniDirDiffThre);
    node_->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
    node_->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
    node_->declare_parameter<double>("inclRateThre", inclRateThre);
    node_->declare_parameter<double>("slowRate1", slowRate1);
    node_->declare_parameter<double>("slowRate2", slowRate2);
    node_->declare_parameter<double>("slowTime1", slowTime1);
    node_->declare_parameter<double>("slowTime2", slowTime2);
    node_->declare_parameter<bool>("useInclToStop", useInclToStop);
    node_->declare_parameter<double>("inclThre", inclThre);
    node_->declare_parameter<double>("stopTime", stopTime);
    node_->declare_parameter<bool>("noRotAtStop", noRotAtStop);
    node_->declare_parameter<bool>("autonomyMode", autonomyMode);
    node_->declare_parameter<double>("autonomySpeed", autonomySpeed);
    node_->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
    node_->declare_parameter<double>("goalYawToleranceDeg", goalYawToleranceDeg);
    node_->declare_parameter<double>("finalYawActivationDist", finalYawActivationDist);
    node_->declare_parameter<double>("finalYawHysteresis", finalYawHysteresis);
    node_->declare_parameter<double>("yawRateFilterAlpha", yawRateFilterAlpha);
    node_->declare_parameter<double>("velFilterAlpha", velFilterAlpha);
    node_->declare_parameter<double>("finalApproachDist", finalApproachDist);
    node_->declare_parameter<double>("finalApproachSpeed", finalApproachSpeed);
    node_->declare_parameter<double>("finalApproachGain", finalApproachGain);
  }

  void loadParameters()
  {
    node_->get_parameter("sensorOffsetX", sensorOffsetX);
    node_->get_parameter("sensorOffsetY", sensorOffsetY);
    node_->get_parameter("pubSkipNum", pubSkipNum);
    node_->get_parameter("twoWayDrive", twoWayDrive);
    node_->get_parameter("lookAheadDis", lookAheadDis);
    node_->get_parameter("yawRateGain", yawRateGain);
    node_->get_parameter("stopYawRateGain", stopYawRateGain);
    node_->get_parameter("maxYawRate", maxYawRate);
    node_->get_parameter("maxYawAccel", maxYawAccel);
    node_->get_parameter("maxSpeed", maxSpeed);
    node_->get_parameter("maxAccel", maxAccel);
    node_->get_parameter("switchTimeThre", switchTimeThre);
    node_->get_parameter("dirDiffThre", dirDiffThre);
    node_->get_parameter("omniDirGoalThre", omniDirGoalThre);
    node_->get_parameter("omniDirDiffThre", omniDirDiffThre);
    node_->get_parameter("slowDwnDisThre", slowDwnDisThre);
    node_->get_parameter("useInclRateToSlow", useInclRateToSlow);
    node_->get_parameter("inclRateThre", inclRateThre);
    node_->get_parameter("slowRate1", slowRate1);
    node_->get_parameter("slowRate2", slowRate2);
    node_->get_parameter("slowTime1", slowTime1);
    node_->get_parameter("slowTime2", slowTime2);
    node_->get_parameter("useInclToStop", useInclToStop);
    node_->get_parameter("inclThre", inclThre);
    node_->get_parameter("stopTime", stopTime);
    node_->get_parameter("noRotAtStop", noRotAtStop);
    node_->get_parameter("autonomyMode", autonomyMode);
    node_->get_parameter("autonomySpeed", autonomySpeed);
    node_->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
    node_->get_parameter("goalYawToleranceDeg", goalYawToleranceDeg);
    node_->get_parameter("finalYawActivationDist", finalYawActivationDist);
    node_->get_parameter("finalYawHysteresis", finalYawHysteresis);
    node_->get_parameter("yawRateFilterAlpha", yawRateFilterAlpha);
    node_->get_parameter("velFilterAlpha", velFilterAlpha);
    node_->get_parameter("finalApproachDist", finalApproachDist);
    node_->get_parameter("finalApproachSpeed", finalApproachSpeed);
    node_->get_parameter("finalApproachGain", finalApproachGain);
    goalYawToleranceRad = goalYawToleranceDeg * PI / 180.0;
  }

  void setupPublishers()
  {
    pubSpeed = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
  }

  void setupSubscriptions()
  {
    subOdom = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 5, std::bind(&PathFollowerNode::odomHandler, this, _1));

    subPath = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", 5, std::bind(&PathFollowerNode::pathHandler, this, _1));

    subRobotCommand = node_->create_subscription<joy_command_mapper::msg::RobotCommand>(
      "/robot_command", 5, std::bind(&PathFollowerNode::robotCommandHandler, this, _1));

    subSpeed = node_->create_subscription<std_msgs::msg::Float32>(
      "/speed", 5, std::bind(&PathFollowerNode::speedHandler, this, _1));

    subStop = node_->create_subscription<std_msgs::msg::Int8>(
      "/stop", 5, std::bind(&PathFollowerNode::stopHandler, this, _1));

    subSlowDown = node_->create_subscription<std_msgs::msg::Int8>(
      "/slow_down", 5, std::bind(&PathFollowerNode::slowDownHandler, this, _1));

    subNavigateCmd = node_->create_subscription<autonomy_msgs::msg::NavigateCmd>(
      "/nav_cmd", 5, std::bind(&PathFollowerNode::navigateCmdHandler, this, _1));
  }

  void initializeAutonomyState()
  {
    if (autonomyMode) {
      joySpeed = autonomySpeed / maxSpeed;

      if (joySpeed < 0) joySpeed = 0;
      else if (joySpeed > 1.0) joySpeed = 1.0;
    }
  }

  void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
  {
    odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    vehicleZ = odomIn->pose.pose.position.z;

    if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
      stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
    }

    if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
      slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
    }
  }

  void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
  {
    int pathSize = pathIn->poses.size();
    path.poses.resize(pathSize);
    for (int i = 0; i < pathSize; i++) {
      path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
      path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
      path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }

    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll;
    vehiclePitchRec = vehiclePitch;
    vehicleYawRec = vehicleYaw;

    pathPointID = 0;
    pathInit = true;
    
    // Reset navigation direction when receiving a new path
    // If twoWayDrive is disabled, ensure we start in forward mode
    if (!twoWayDrive) {
      navFwd = true;
    }
  }

  void robotCommandHandler(const joy_command_mapper::msg::RobotCommand::ConstSharedPtr cmd)
  {
    joyTime = rclcpp::Time(cmd->header.stamp).seconds();
    joySpeedRaw = sqrt(cmd->vy * cmd->vy + cmd->vx * cmd->vx);
    joySpeed = joySpeedRaw;
    if (joySpeed > 1.0) joySpeed = 1.0;
    if (cmd->vx == 0) joySpeed = 0;
    joyYaw = cmd->vy;
    if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

    if (cmd->vx < 0 && !twoWayDrive) {
      joySpeed = 0;
      joyYaw = 0;
    }

    joyManualFwd = cmd->vx;
    joyManualLeft = cmd->vy;
    joyManualYaw = cmd->wz;

    autonomyMode = cmd->autonomy_mode;
    manualMode = cmd->manual_mode;
  }

  void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
  {
    double speedTime = node_->now().seconds();
    if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
      joySpeed = speed->data / maxSpeed;

      if (joySpeed < 0) joySpeed = 0;
      else if (joySpeed > 1.0) joySpeed = 1.0;
    }
  }

  void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
  {
    safetyStop = stop->data;
  }

  void slowDownHandler(const std_msgs::msg::Int8::ConstSharedPtr slow)
  {
    slowDown = slow->data;
  }

  void navigateCmdHandler(const autonomy_msgs::msg::NavigateCmd::ConstSharedPtr nav_cmd)
  {
    const geometry_msgs::msg::Quaternion &orientation = nav_cmd->goal_pose.pose.orientation;
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    if (nav_cmd->track_yaw && quat.length2() > 1e-6) {
      quat.normalize();
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      goalYaw = yaw;
      goalYawValid = true;
    } else {
      goalYawValid = false;
    }

    // Store goal position in world frame
    goalX = nav_cmd->goal_pose.pose.position.x;
    goalY = nav_cmd->goal_pose.pose.position.y;
    goalPositionValid = true;
    finalApproachMode = 0;  // Reset to NORMAL mode

    joySpeed = 1.0;
    joyTime = rclcpp::Time(nav_cmd->goal_pose.header.stamp).seconds();

    autonomyMode = nav_cmd->is_autonomy;
    twoWayDrive = nav_cmd->allow_reverse;

    // Reset navigation direction state when receiving new command
    // If reverse is not allowed, ensure we start in forward mode
    if (!twoWayDrive) {
      navFwd = true;
    }
  }

  void processControlStep()
  {
    if (!pathInit) {
      return;
    }

    float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                      + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
    float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                      + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

    int pathSize = path.poses.size();
    float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
    float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
    float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

    float disX, disY, dis;
    while (pathPointID < pathSize - 1) {
      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      if (dis < lookAheadDis) {
        pathPointID++;
      } else {
        break;
      }
    }

    disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
    disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
    dis = sqrt(disX * disX + disY * disY);
    float pathDir = atan2(disY, disX);

    // Calculate distance to actual goal in world frame
    float goalDisX = goalX - vehicleX;
    float goalDisY = goalY - vehicleY;
    float goalDis = sqrt(goalDisX * goalDisX + goalDisY * goalDisY);

    double yawError = 0.0;
    // Final approach mode state machine (only active in autonomy mode with valid goal)
    // Mode 0: NORMAL - regular path following
    // Mode 1: FINAL_XY_APPROACH - move with x/y velocity toward goal (no yaw rotation)
    // Mode 2: FINAL_YAW_ADJUST - only adjust yaw (no translation)
    if (!autonomyMode) {
      // Reset to normal mode when not in autonomy
      finalApproachMode = 0;
    } else {
      yawError = goalYawValid ? atan2(sin(goalYaw - vehicleYaw), cos(goalYaw - vehicleYaw)) : 0.0;
      double absYawError = fabs(yawError);

      bool goalInYawAdjustDis = goalDis <= finalYawActivationDist;
      bool goalInFinalApproachDis = goalDis <= finalApproachDist;
      bool allowOmni = omniDirGoalThre > 0.0;
      bool yawInTolerance = absYawError <= goalYawToleranceRad;

      if (finalApproachMode == 0) {
        // NORMAL mode: check if we should enter FINAL_XY_APPROACH or skip directly to YAW_ADJUST
        if (goalInYawAdjustDis && goalYawValid && !yawInTolerance) {
          finalApproachMode = 2;
          RCLCPP_INFO(node_->get_logger(), "Final approach: direct YAW mode (goal already close), goalDis=%.2f m, yawError=%.2f rad", goalDis, yawError);
        } else if (goalInFinalApproachDis && allowOmni) {
          finalApproachMode = 1;
          RCLCPP_INFO(node_->get_logger(), "Final approach: entering XY mode, goalDis=%.2f m", goalDis);
        }
      } else if (finalApproachMode == 1) {
        // FINAL_XY_APPROACH mode: move toward goal with x/y velocity only
        if (goalInYawAdjustDis) {
          // In goal clear range, switch to yaw adjustment if needed
          if (goalYawValid && !yawInTolerance) {
            finalApproachMode = 2;
            RCLCPP_INFO(node_->get_logger(), "Final approach: entering YAW mode, yawError=%.2f rad", yawError);
          } else {
            // Goal reached, no yaw adjustment needed
            finalApproachMode = 0;
            RCLCPP_INFO(node_->get_logger(), "Final approach: goal reached");
          }
        } else if (goalDis > finalApproachDist * finalYawHysteresis) {
          // Robot moved too far from goal, return to normal mode
          finalApproachMode = 0;
          RCLCPP_INFO(node_->get_logger(), "Final approach: exiting XY mode (too far), goalDis=%.2f m", goalDis);
        }
      } else if (finalApproachMode == 2) {
        if (absYawError <= goalYawToleranceRad / finalYawHysteresis) {
          // Yaw adjustment complete
          RCLCPP_INFO(node_->get_logger(), "Final approach: yaw adjustment complete");
        } else if (goalDis > finalYawActivationDist * finalYawHysteresis) {
          // Robot moved too far, go back to XY approach
          if (allowOmni) finalApproachMode = 0;
          else finalApproachMode = 1;
          RCLCPP_INFO(node_->get_logger(), "Final approach: re-entering XY mode, goalDis=%.2f m, finalApproachMode=%d", goalDis, finalApproachMode);
        }
      }
    } 

    float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
    if (dirDiff > PI) dirDiff -= 2 * PI;
    else if (dirDiff < -PI) dirDiff += 2 * PI;
    if (dirDiff > PI) dirDiff -= 2 * PI;
    else if (dirDiff < -PI) dirDiff += 2 * PI;

    if (twoWayDrive) {
      double time = node_->now().seconds();
      if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
        navFwd = false;
        switchTime = time;
      } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
        navFwd = true;
        switchTime = time;
      }
    }

    float joySpeed2 = maxSpeed * joySpeed;
    if (!navFwd) {
      dirDiff += PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      joySpeed2 *= -1;
    }

    if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) targetYawRate = -stopYawRateGain * dirDiff;
    else targetYawRate = -yawRateGain * dirDiff;

    if (targetYawRate > maxYawRate * PI / 180.0) targetYawRate = maxYawRate * PI / 180.0;
    else if (targetYawRate < -maxYawRate * PI / 180.0) targetYawRate = -maxYawRate * PI / 180.0;

    if (joySpeed2 == 0 && !autonomyMode) {
      targetYawRate = maxYawRate * joyYaw * PI / 180.0;
    }

    // In FINAL_XY_APPROACH mode, disable yaw rotation
    if (finalApproachMode == 1) {
      targetYawRate = 0;
    }

    if (pathSize <= 1) {
      joySpeed2 = 0;
    } else if (endDis / slowDwnDisThre < joySpeed) {
      joySpeed2 *= endDis / slowDwnDisThre;
    }

    float joySpeed3 = joySpeed2;
    if ((odomTime < slowInitTime + slowTime1 && slowInitTime > 0) || slowDown == 1) joySpeed3 *= slowRate1;
    else if ((odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) || slowDown == 2) joySpeed3 *= slowRate2;

    if (finalApproachMode == 0) {
      // NORMAL mode: regular path following speed control
      if ((fabs(dirDiff) < dirDiffThre || (dis < omniDirGoalThre && fabs(dirDiff) < omniDirDiffThre))) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }
    } else if (finalApproachMode == 1) {
      // FINAL_XY_APPROACH mode: speed is controlled by P-gain toward goal
      // vehicleSpeed will be used differently - we compute velocity in publishing section
      vehicleSpeed = 0;  // Not used in XY approach mode
    } else if (finalApproachMode == 2) {
      // FINAL_YAW_ADJUST mode: no translation
      vehicleSpeed = 0;
      targetYawRate = stopYawRateGain * yawError;
      double maxYawRateRad = maxYawRate * PI / 180.0;
      if (targetYawRate > maxYawRateRad) {targetYawRate = maxYawRateRad;}
      else if (targetYawRate < -maxYawRateRad) {targetYawRate = -maxYawRateRad;}

    } else {
      RCLCPP_ERROR(node_->get_logger(), "Invalid pathFollower state");
      vehicleSpeed = 0;
      targetYawRate = 0;
    }

    if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
      vehicleSpeed = 0;
      targetYawRate = 0;
    }

    if (safetyStop >= 1) vehicleSpeed = 0;
    if (safetyStop >= 2) targetYawRate = 0;

    // Low-pass filter on target yaw rate to reduce oscillations from state estimation noise
    filteredTargetYawRate = yawRateFilterAlpha * targetYawRate + (1.0 - yawRateFilterAlpha) * filteredTargetYawRate;

    // Gradual yaw rate acceleration
    float yawRateErr = vehicleYawRate - filteredTargetYawRate;
    if (yawRateErr < -0.1) vehicleYawRate += maxYawAccel * PI / 180.0 / 100.0;
    else if (yawRateErr > 0.1) vehicleYawRate -= maxYawAccel * PI / 180.0 / 100.0;
    else vehicleYawRate = filteredTargetYawRate;

    // RCLCPP_INFO(node_->get_logger(), "finalApproachMode: %d, goalYawValid: %d, goalYaw: %.2f, goalDist: %.2f, yawError: %.2f, vehicleYaw: %.2f, targetYawRate: %.2f, vehicleYawRate: %.2f, yawRateErr: %.2f",
    //   finalApproachMode, goalYawValid, goalYaw, dis, yawError, vehicleYaw, targetYawRate, vehicleYawRate, yawRateErr);

    // RCLCPP_INFO(node_->get_logger(), "pathSize: %d, pathPointID: %d, navFwd: %d, dis: %.2f, endDis: %.2f, dirDiff: %.2f, joySpeed2: %.2f, joySpeed3: %.2f, vehicleSpeed: %.2f, targetYawRate: %.2f",
    //   pathSize, pathPointID, navFwd, dis, endDis, dirDiff, joySpeed2, joySpeed3, vehicleSpeed, targetYawRate);


    pubSkipCount--;
    if (pubSkipCount < 0) {
      cmdVel_.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      cmdVel_.twist.linear.x = 0;
      cmdVel_.twist.linear.y = 0;
      cmdVel_.twist.angular.z = vehicleYawRate;

      if (finalApproachMode == 1) {
        // FINAL_XY_APPROACH mode: compute velocities toward goal in body frame
        // Transform goal direction from world to body frame
        float goalDirWorld = atan2(goalDisY, goalDisX);
        float goalDirBody = goalDirWorld - vehicleYaw;
        // Normalize to [-PI, PI]
        if (goalDirBody > PI) goalDirBody -= 2 * PI;
        else if (goalDirBody < -PI) goalDirBody += 2 * PI;

        // Compute velocity with P-gain control, capped at finalApproachSpeed
        float approachSpeed = finalApproachGain * goalDis;
        if (approachSpeed > finalApproachSpeed) approachSpeed = finalApproachSpeed;

        float desiredVelX = cos(goalDirBody) * approachSpeed;
        float desiredVelY = sin(goalDirBody) * approachSpeed;

        // Apply low-pass filter for smoother motion
        filteredVelX = velFilterAlpha * desiredVelX + (1.0 - velFilterAlpha) * filteredVelX;
        filteredVelY = velFilterAlpha * desiredVelY + (1.0 - velFilterAlpha) * filteredVelY;
        cmdVel_.twist.linear.x = filteredVelX;
        cmdVel_.twist.linear.y = filteredVelY;
        cmdVel_.twist.angular.z = 0;  // No yaw rotation in XY approach mode
      } else if (fabs(vehicleSpeed) > maxAccel / 100.0) {
        if (omniDirGoalThre > 0) {
          // Compute desired velocities
          float desiredVelX = cos(dirDiff) * vehicleSpeed;
          float desiredVelY = -sin(dirDiff) * vehicleSpeed;
          // Apply low-pass filter for smoother direction changes
          filteredVelX = velFilterAlpha * desiredVelX + (1.0 - velFilterAlpha) * filteredVelX;
          filteredVelY = velFilterAlpha * desiredVelY + (1.0 - velFilterAlpha) * filteredVelY;
          cmdVel_.twist.linear.x = filteredVelX;
          cmdVel_.twist.linear.y = filteredVelY;
        } else {
          cmdVel_.twist.linear.x = vehicleSpeed;
        }
      } else {
        // Reset filtered velocities when stopped to avoid drift on restart
        filteredVelX = 0;
        filteredVelY = 0;
      }

      if (manualMode) {
        cmdVel_.twist.linear.x = maxSpeed * joyManualFwd;
        if (omniDirGoalThre > 0) cmdVel_.twist.linear.y = maxSpeed / 2.0 * joyManualLeft;
        cmdVel_.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;
      }

      pubSpeed->publish(cmdVel_);
      pubSkipCount = pubSkipNum;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pathFollower");
  auto follower = std::make_shared<PathFollowerNode>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
