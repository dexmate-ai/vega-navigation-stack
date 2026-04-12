#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <joy_command_mapper/msg/robot_command.hpp>
#include <autonomy_msgs/msg/navigate_cmd.hpp>
#include <autonomy_msgs/msg/goal_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

const double PI = 3.1415926;

#define PLOTPATHSET 1

// LocalPlannerNode encapsulates pointcloud processing, path scoring, and ROS I/O that
// were previously handled through globals, keeping all planner state within a single class.
class LocalPlannerNode
{
public:
  explicit LocalPlannerNode(const rclcpp::Node::SharedPtr &node)
  : node_(node)
  {
    declareParameters();
    loadParameters();
    initializePointStorage();
    configureFilters();
    setupPublishers();
    setupSubscriptions();
    setupServices();
    readStaticData();
    initializeAutonomyState();
    controlTimer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&LocalPlannerNode::processPlannerCycle, this));
    updateNavStatus(false, false);
    RCLCPP_INFO(node_->get_logger(), "Initialization complete.");
  }

private:
  static constexpr int pathNum = 343;
  static constexpr int groupNum = 7;
  static constexpr int laserCloudStackNum = 1;
  static constexpr int rotationSampleCount = 36;
  static constexpr int gridVoxelNumX = 161;
  static constexpr int gridVoxelNumY = 451;
  static constexpr int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;
  static constexpr int noPathCycleThreshold = 300;

  enum NavStatusBits : int32_t {
    NAV_STATUS_ACTIVE = 1 << 0,
    NAV_STATUS_GOAL_REACHED = 1 << 2,
    NAV_STATUS_GOAL_STUCK = 1 << 3,
    NAV_STATUS_MANUAL = 1 << 4,
  };

  using CloudI = pcl::PointCloud<pcl::PointXYZI>;
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  rclcpp::Node::SharedPtr node_;

  // Parameters
  string pathFolder;
  double vehicleLength = 0.6;
  double vehicleWidth = 0.6;
  bool twoWayDrive = true;
  double laserVoxelSize = 0.05;
  double terrainVoxelSize = 0.2;
  bool checkObstacle = true;
  bool checkRotObstacle = false;
  double adjacentRange = 3.5;
  double obstacleHeightThre = 0.2;
  double groundHeightThre = 0.1;
  double costHeightThre1 = 0.15;
  double costHeightThre2 = 0.1;
  bool useCost = false;
  int pointPerPathThre = 2;
  double maxSpeed = 1.0;
  double dirWeight = 0.02;
  double dirThre = 90.0;
  bool dirToVehicle = false;
  double pathScale = 1.0;
  double minPathScale = 0.75;
  double pathScaleStep = 0.25;
  bool pathScaleBySpeed = true;
  double minPathRange = 1.0;
  double pathRangeStep = 0.5;
  bool pathRangeBySpeed = true;
  bool pathCropByGoal = true;
  bool autonomyMode = false;
  double autonomySpeed = 1.0;
  double joyToSpeedDelay = 2.0;
  double joyToCheckObstacleDelay = 5.0;
  double freezeAng = 90.0;
  double freezeTime = 2.0;
  double omniDirGoalThre = 1.0;
  double goalClearRange = 0.5;
  double goalClearRangeHysteresis = 1.5;
  double goalBehindRange = 0.8;
  float gridVoxelSize = 0.02;
  float searchRadius = 0.45;
  float gridVoxelOffsetX = 3.2;
  float gridVoxelOffsetY = 4.5;
  std::string registeredScanTopic = "/registered_scan_filtered_map";

  pcl::PointXYZ goalWorld;
  double goalYaw = 0.0;
  bool goalYawValid = false;
  double goalYawToleranceDeg = 5.0;
  double goalYawToleranceRad = goalYawToleranceDeg * PI / 180.0;

  // Dynamic state
  int laserCloudCount = 0;
  double freezeStartTime = 0;
  int freezeStatus = 0;
  float joySpeed = 0;
  float joySpeedRaw = 0;
  float joyDir = 0;
  bool newLaserCloud = false;
  bool newTerrainCloud = false;
  double odomTime = 0;
  double joyTime = 0;
  float vehicleRoll = 0;
  float vehiclePitch = 0;
  float vehicleYaw = 0;
  float vehicleX = 0;
  float vehicleY = 0;
  float vehicleZ = 0;

  // Point cloud storage
  CloudI::Ptr laserCloud;
  CloudI::Ptr laserCloudCrop;
  CloudI::Ptr laserCloudDwz;
  CloudI::Ptr terrainCloud;
  CloudI::Ptr terrainCloudCrop;
  CloudI::Ptr terrainCloudDwz;
  std::array<CloudI::Ptr, laserCloudStackNum> laserCloudStack;
  CloudI::Ptr plannerCloud;
  CloudI::Ptr plannerCloudCrop;
  CloudI::Ptr boundaryCloud;
  CloudI::Ptr addedObstacles;
  std::array<Cloud::Ptr, groupNum> startPaths;
#if PLOTPATHSET == 1
  std::array<CloudI::Ptr, pathNum> paths;
  CloudI::Ptr freePaths;
#endif

  // Path selection storage
  std::array<int, pathNum> pathList{};
  std::array<float, pathNum> endDirPathList{};
  std::array<std::vector<int>, gridVoxelNum> correspondences;

  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

  /**
   * @brief Reset per-path and per-group accumulators prior to scoring.
   *
   * @param[out] clearPathListRef Collision counters for every (rotation, path) candidate.
   * @param[out] pathPenaltyListRef Soft obstacle penalties for every (rotation, path) candidate.
   * @param[out] clearPathPerGroupScoreRef Aggregated scores per (rotation, group) pairing.
   * @param[out] clearPathPerGroupNumRef Number of contributing paths per (rotation, group).
   * @param[out] pathPenaltyPerGroupScoreRef Aggregated soft penalties per (rotation, group).
   */
  void resetPathStatistics(
    std::array<int, rotationSampleCount * pathNum> &clearPathListRef,
    std::array<float, rotationSampleCount * pathNum> &pathPenaltyListRef,
    std::array<float, rotationSampleCount * groupNum> &clearPathPerGroupScoreRef,
    std::array<int, rotationSampleCount * groupNum> &clearPathPerGroupNumRef,
    std::array<float, rotationSampleCount * groupNum> &pathPenaltyPerGroupScoreRef) const
  {
    for (int i = 0; i < rotationSampleCount * pathNum; i++) {
      clearPathListRef[i] = 0;
      pathPenaltyListRef[i] = 0;
    }
    for (int i = 0; i < rotationSampleCount * groupNum; i++) {
      clearPathPerGroupScoreRef[i] = 0;
      clearPathPerGroupNumRef[i] = 0;
      pathPenaltyPerGroupScoreRef[i] = 0;
    }
  }

  /**
   * @brief Accumulate obstacle statistics against every candidate path.
   *
   * @param[in] pathScale Current template scale used for collision checks.
   * @param[in] pathRange Maximum look-ahead range for candidate evaluation.
   * @param[in] relativeGoalDis Distance to the goal in vehicle coordinates.
   * @param[in] joyDir Desired travel direction in degrees.
   * @param[out] minObsAngCW Minimum clockwise steering angle still considered clear.
   * @param[out] minObsAngCCW Minimum counter-clockwise steering angle still considered clear.
   * @param[in,out] clearPathListRef Collision counters to update.
   * @param[in,out] pathPenaltyListRef Soft obstacle penalties to update.
   */
  void gatherObstacleStatistics(
    float pathScale, float pathRange, float relativeGoalDis, float joyDir,
    float &minObsAngCW, float &minObsAngCCW,
    std::array<int, rotationSampleCount * pathNum> &clearPathListRef,
    std::array<float, rotationSampleCount * pathNum> &pathPenaltyListRef) const
  {
    float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
    float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
    int plannerCloudCropSize = plannerCloudCrop->points.size();

    for (int i = 0; i < plannerCloudCropSize; i++) {
      float x = plannerCloudCrop->points[i].x / pathScale;
      float y = plannerCloudCrop->points[i].y / pathScale;
      float h = plannerCloudCrop->points[i].intensity;
      float dis = sqrt(x * x + y * y);

      if (dis < pathRange / pathScale && (dis <= (relativeGoalDis +  goalClearRange * 0.6) / pathScale || !pathCropByGoal) && checkObstacle) {
        for (int rotDir = 0; rotDir < rotationSampleCount; rotDir++) {
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }

          float x2 = cos(rotAng) * x + sin(rotAng) * y;
          float y2 = -sin(rotAng) * x + cos(rotAng) * y;

          float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY
                         * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

          int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
          int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
          if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
            int ind = gridVoxelNumY * indX + indY;
            int blockedPathByVoxelNum = correspondences[ind].size();
            for (int j = 0; j < blockedPathByVoxelNum; j++) {
              if (h > obstacleHeightThre) {
                clearPathListRef[pathNum * rotDir + correspondences[ind][j]]++;
              }
              if (useCost && h > groundHeightThre && h <= obstacleHeightThre) {
                pathPenaltyListRef[pathNum * rotDir + correspondences[ind][j]] += h;
              }
            }
          }

          float angObs = atan2(y2, x2) * 180.0 / PI;
          if (angObs > 180.0) angObs -= 360.0;
          if (angObs < -180.0) angObs += 360.0;
          if (dis < diameter) {
            minObsAngCW = 180.0;
            minObsAngCCW = -180.0;
          } else if (angObs >= 0) {
            if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
            if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
          } else {
            if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
            if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
          }
        }
      }
    }

    if (minObsAngCW > 0) minObsAngCW = 0;
    if (minObsAngCCW < 0) minObsAngCCW = 0;
  }

  /**
   * @brief Compute path scores and select the leading (rotation, group) candidate.
   *
   * @param[in] relativeGoalDis Distance to the goal in vehicle coordinates.
   * @param[in] joyDir Desired travel direction in degrees.
   * @param[in] minObsAngCW Minimum clockwise steering angle still considered clear.
   * @param[in] minObsAngCCW Minimum counter-clockwise steering angle still considered clear.
   * @param[out] penaltyScore Averaged soft obstacle penalty for the winning candidate.
   * @param[in] clearPathListRef Collision counters per (rotation, path).
   * @param[in] pathPenaltyListRef Soft penalties per (rotation, path).
   * @param[in,out] clearPathPerGroupScoreRef Accumulated scores per (rotation, group).
   * @param[in,out] clearPathPerGroupNumRef Contributor counts per (rotation, group).
   * @param[in,out] pathPenaltyPerGroupScoreRef Accumulated soft penalties per (rotation, group).
   * @return Index of the best (rotation, group) combination, or -1 if none.
   */
  int evaluatePathScores(
    float relativeGoalDis, float joyDir, float minObsAngCW, float minObsAngCCW,
    float &penaltyScore,
    const std::array<int, rotationSampleCount * pathNum> &clearPathListRef,
    const std::array<float, rotationSampleCount * pathNum> &pathPenaltyListRef,
    std::array<float, rotationSampleCount * groupNum> &clearPathPerGroupScoreRef,
    std::array<int, rotationSampleCount * groupNum> &clearPathPerGroupNumRef,
    std::array<float, rotationSampleCount * groupNum> &pathPenaltyPerGroupScoreRef) const
  {
    for (int i = 0; i < rotationSampleCount * pathNum; i++) {
      int rotDir = int(i / pathNum);
      float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
      if (angDiff > 180.0) {
        angDiff = 360.0 - angDiff;
      }
      if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
          ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
        continue;
      }

      if (clearPathListRef[i] < pointPerPathThre) {
        float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
        if (dirDiff > 360.0) {
          dirDiff -= 360.0;
        }
        if (dirDiff > 180.0) {
          dirDiff = 360.0 - dirDiff;
        }

        float rotDirW;
        if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
        else rotDirW = fabs(fabs(rotDir - 27) + 1);
        float groupDirW = 4  - fabs(pathList[i % pathNum] - 3);
        float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW;
        if (relativeGoalDis < omniDirGoalThre) score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW * groupDirW;
        if (score > 0) {
          clearPathPerGroupScoreRef[groupNum * rotDir + pathList[i % pathNum]] += score;
          clearPathPerGroupNumRef[groupNum * rotDir + pathList[i % pathNum]]++;
          pathPenaltyPerGroupScoreRef[groupNum * rotDir + pathList[i % pathNum]] += pathPenaltyListRef[i];
        }
      }
    }

    float maxScore = 0;
    int selectedGroupID = -1;
    for (int i = 0; i < rotationSampleCount * groupNum; i++) {
      int rotDir = int(i / groupNum);
      float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
      float rotDeg = 10.0 * rotDir;
      if (rotDeg > 180.0) rotDeg -= 360.0;
      if (maxScore < clearPathPerGroupScoreRef[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
          (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
        maxScore = clearPathPerGroupScoreRef[i];
        selectedGroupID = i;
      }
    }

    penaltyScore = 0;
    if (selectedGroupID >= 0 && clearPathPerGroupNumRef[selectedGroupID] > 0) {
      penaltyScore = pathPenaltyPerGroupScoreRef[selectedGroupID] / clearPathPerGroupNumRef[selectedGroupID];
    }

    return selectedGroupID;
  }

  /**
   * @brief Publish a slow-down level based on average path penalty.
   *
   * @param[in] penaltyScore Average soft obstacle penalty for the winning group.
   */
  void publishSlowDownCommand(float penaltyScore)
  {
    if (penaltyScore > costHeightThre1) slowMsg_.data = 1;
    else if (penaltyScore > costHeightThre2) slowMsg_.data = 2;
    else slowMsg_.data = 0;
    pubSlowDown->publish(slowMsg_);
  }

  /**
   * @brief Publish stop command to pathFollower.
   *
   * @param[in] goalReached Whether goal position and orientation are reached.
   */
  void publishStopCommand(bool goalReached)
  {
    if (goalReached) {
      stopMsg_.data = 2;  // Stop both translation and rotation
      goalStopLatched_ = true;
    } else if (goalStopLatched_) {
      stopMsg_.data = 2;  // Maintain stop until a new goal arrives
    } else {
      stopMsg_.data = 0;  // No stop
    }
    pubStop->publish(stopMsg_);
  }

  /**
   * @brief Publish the selected path (and optionally the free path cloud) with current scaling.
   *
   * @param[in] selectedGroupID Winning (rotation, group) index.
   * @param[in] pathScale Applied template scale.
   * @param[in] pathRange Applied look-ahead range.
   * @param[in] relativeGoalDis Distance to the goal in vehicle coordinates.
   * @param[in] joyDir Desired travel direction in degrees.
   * @param[in] minObsAngCW Minimum clockwise steering angle still considered clear.
   * @param[in] minObsAngCCW Minimum counter-clockwise steering angle still considered clear.
   * @param[in] clearPathListRef Collision counters used for optional free-path visualization.
   * @return true if a path was published, false otherwise.
   */
  bool publishSelectedPath(
    int selectedGroupID, float pathScale, float pathRange, float relativeGoalDis,
    float joyDir, float minObsAngCW, float minObsAngCCW,
    const std::array<int, rotationSampleCount * pathNum> &clearPathListRef)
  {
    int rotDir = int(selectedGroupID / groupNum);
    float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

    selectedGroupID = selectedGroupID % groupNum;
    int selectedPathLength = startPaths[selectedGroupID]->points.size();
    pathMsg_.poses.resize(selectedPathLength);
    for (int i = 0; i < selectedPathLength; i++) {
      float x = startPaths[selectedGroupID]->points[i].x;
      float y = startPaths[selectedGroupID]->points[i].y;
      float z = startPaths[selectedGroupID]->points[i].z;
      float dis = sqrt(x * x + y * y);

      if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
        pathMsg_.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
        pathMsg_.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
        pathMsg_.poses[i].pose.position.z = pathScale * z;
      } else {
        pathMsg_.poses.resize(i);
        break;
      }
    }

    pathMsg_.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
    pathMsg_.header.frame_id = "vehicle";
    pubPath->publish(pathMsg_);

#if PLOTPATHSET == 1
    freePaths->clear();
    for (int i = 0; i < rotationSampleCount * pathNum; i++) {
      int rotDirAll = int(i / pathNum);
      float rotAngAll = (10.0 * rotDirAll - 180.0) * PI / 180;
      float rotDegAll = 10.0 * rotDirAll;
      if (rotDegAll > 180.0) rotDegAll -= 360.0;
      float angDiff = fabs(joyDir - (10.0 * rotDirAll - 180.0));
      if (angDiff > 180.0) {
        angDiff = 360.0 - angDiff;
      }
      if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDirAll - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
          ((10.0 * rotDirAll > dirThre && 360.0 - 10.0 * rotDirAll > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) ||
          !((rotAngAll * 180.0 / PI > minObsAngCW && rotAngAll * 180.0 / PI < minObsAngCCW) ||
          (rotDegAll > minObsAngCW && rotDegAll < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
        continue;
      }

      if (clearPathListRef[i] < pointPerPathThre) {
        int freePathLength = paths[i % pathNum]->points.size();
        for (int j = 0; j < freePathLength; j++) {
          pcl::PointXYZI point = paths[i % pathNum]->points[j];

          float x = point.x;
          float y = point.y;
          float z = point.z;

          float dis = sqrt(x * x + y * y);
          if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange * 0.8) / pathScale || !pathCropByGoal)) {
            point.x = pathScale * (cos(rotAngAll) * x - sin(rotAngAll) * y);
            point.y = pathScale * (sin(rotAngAll) * x + cos(rotAngAll) * y);
            point.z = pathScale * z;
            point.intensity = 1.0;

            freePaths->push_back(point);
          }
        }
      }
    }

    sensor_msgs::msg::PointCloud2 freePaths2;
    pcl::toROSMsg(*freePaths, freePaths2);
    freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
    freePaths2.header.frame_id = "vehicle";
    pubFreePaths->publish(freePaths2);
#endif

    return true;
  }

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloud;
  rclcpp::Subscription<joy_command_mapper::msg::RobotCommand>::SharedPtr subRobotCommand;
  rclcpp::Subscription<autonomy_msgs::msg::NavigateCmd>::SharedPtr subNavigateCmd;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subBoundary;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subAddedObstacles;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subCheckObstacle;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubSlowDown;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubStop;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
#if PLOTPATHSET == 1
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreePaths;
#endif
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr navStatusPub_;
  rclcpp::Publisher<autonomy_msgs::msg::GoalStatus>::SharedPtr goalStatusPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCurrentGoal;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr navStatusService_;
  rclcpp::TimerBase::SharedPtr controlTimer_;
  std_msgs::msg::Int8 slowMsg_;
  std_msgs::msg::Int8 stopMsg_;
  nav_msgs::msg::Path pathMsg_;
  std_msgs::msg::Int32 navStatusMsg_;
  autonomy_msgs::msg::GoalStatus goalStatusMsg_;
  int32_t navStatus_ = 0;
  int noPathCyclesWithoutPath_ = 0;
  bool lastGoalReached_ = false;
  bool lastGoalStuck_ = false;
  bool goalWithinClearRange_ = false;
  bool goalStopLatched_ = false;
  bool goalReachedLatched_ = false;

  void declareParameters()
  {
    node_->declare_parameter<std::string>("pathFolder", pathFolder);
    node_->declare_parameter<double>("vehicleLength", vehicleLength);
    node_->declare_parameter<double>("vehicleWidth", vehicleWidth);
    node_->declare_parameter<bool>("twoWayDrive", twoWayDrive);
    node_->declare_parameter<double>("laserVoxelSize", laserVoxelSize);
    node_->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
    node_->declare_parameter<bool>("checkObstacle", checkObstacle);
    node_->declare_parameter<bool>("checkRotObstacle", checkRotObstacle);
    node_->declare_parameter<double>("adjacentRange", adjacentRange);
    node_->declare_parameter<double>("obstacleHeightThre", obstacleHeightThre);
    node_->declare_parameter<double>("groundHeightThre", groundHeightThre);
    node_->declare_parameter<double>("costHeightThre1", costHeightThre1);
    node_->declare_parameter<double>("costHeightThre2", costHeightThre2);
    node_->declare_parameter<bool>("useCost", useCost);
    node_->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
    node_->declare_parameter<double>("maxSpeed", maxSpeed);
    node_->declare_parameter<double>("dirWeight", dirWeight);
    node_->declare_parameter<double>("dirThre", dirThre);
    node_->declare_parameter<bool>("dirToVehicle", dirToVehicle);
    node_->declare_parameter<double>("pathScale", pathScale);
    node_->declare_parameter<double>("minPathScale", minPathScale);
    node_->declare_parameter<double>("pathScaleStep", pathScaleStep);
    node_->declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed);
    node_->declare_parameter<double>("minPathRange", minPathRange);
    node_->declare_parameter<double>("pathRangeStep", pathRangeStep);
    node_->declare_parameter<bool>("pathRangeBySpeed", pathRangeBySpeed);
    node_->declare_parameter<bool>("pathCropByGoal", pathCropByGoal);
    node_->declare_parameter<bool>("autonomyMode", autonomyMode);
    node_->declare_parameter<double>("autonomySpeed", autonomySpeed);
    node_->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
    node_->declare_parameter<double>("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
    node_->declare_parameter<double>("freezeAng", freezeAng);
    node_->declare_parameter<double>("freezeTime", freezeTime);
    node_->declare_parameter<double>("omniDirGoalThre", omniDirGoalThre);
    node_->declare_parameter<double>("goalClearRange", goalClearRange);
    node_->declare_parameter<double>("goalClearRangeHysteresis", goalClearRangeHysteresis);
    node_->declare_parameter<double>("goalBehindRange", goalBehindRange);
    node_->declare_parameter<double>("goalYawToleranceDeg", goalYawToleranceDeg);
    node_->declare_parameter<float>("gridVoxelSize", gridVoxelSize);
    node_->declare_parameter<float>("searchRadius", searchRadius);
    node_->declare_parameter<float>("gridVoxelOffsetX", gridVoxelOffsetX);
    node_->declare_parameter<float>("gridVoxelOffsetY", gridVoxelOffsetY);
    node_->declare_parameter<std::string>("registered_scan_topic", registeredScanTopic);
  }

  void loadParameters()
  {
    node_->get_parameter("pathFolder", pathFolder);
    node_->get_parameter("vehicleLength", vehicleLength);
    node_->get_parameter("vehicleWidth", vehicleWidth);
    node_->get_parameter("twoWayDrive", twoWayDrive);
    node_->get_parameter("laserVoxelSize", laserVoxelSize);
    node_->get_parameter("terrainVoxelSize", terrainVoxelSize);
    node_->get_parameter("checkObstacle", checkObstacle);
    node_->get_parameter("checkRotObstacle", checkRotObstacle);
    node_->get_parameter("adjacentRange", adjacentRange);
    node_->get_parameter("obstacleHeightThre", obstacleHeightThre);
    node_->get_parameter("groundHeightThre", groundHeightThre);
    node_->get_parameter("costHeightThre1", costHeightThre1);
    node_->get_parameter("costHeightThre2", costHeightThre2);
    node_->get_parameter("useCost", useCost);
    node_->get_parameter("pointPerPathThre", pointPerPathThre);
    node_->get_parameter("maxSpeed", maxSpeed);
    node_->get_parameter("dirWeight", dirWeight);
    node_->get_parameter("dirThre", dirThre);
    node_->get_parameter("dirToVehicle", dirToVehicle);
    node_->get_parameter("pathScale", pathScale);
    node_->get_parameter("minPathScale", minPathScale);
    node_->get_parameter("pathScaleStep", pathScaleStep);
    node_->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
    node_->get_parameter("minPathRange", minPathRange);
    node_->get_parameter("pathRangeStep", pathRangeStep);
    node_->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
    node_->get_parameter("pathCropByGoal", pathCropByGoal);
    node_->get_parameter("autonomyMode", autonomyMode);
    node_->get_parameter("autonomySpeed", autonomySpeed);
    node_->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
    node_->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
    node_->get_parameter("freezeAng", freezeAng);
    node_->get_parameter("freezeTime", freezeTime);
    node_->get_parameter("omniDirGoalThre", omniDirGoalThre);
    node_->get_parameter("goalClearRange", goalClearRange);
    node_->get_parameter("goalClearRangeHysteresis", goalClearRangeHysteresis);
    node_->get_parameter("goalBehindRange", goalBehindRange);
    node_->get_parameter("goalYawToleranceDeg", goalYawToleranceDeg);
    goalYawToleranceRad = goalYawToleranceDeg * PI / 180.0;
    node_->get_parameter("gridVoxelSize", gridVoxelSize);
    node_->get_parameter("searchRadius", searchRadius);
    node_->get_parameter("gridVoxelOffsetX", gridVoxelOffsetX);
    node_->get_parameter("gridVoxelOffsetY", gridVoxelOffsetY);
    node_->get_parameter("registered_scan_topic", registeredScanTopic);
  }

  void initializePointStorage()
  {
    laserCloud = std::make_shared<CloudI>();
    laserCloudCrop = std::make_shared<CloudI>();
    laserCloudDwz = std::make_shared<CloudI>();
    terrainCloud = std::make_shared<CloudI>();
    terrainCloudCrop = std::make_shared<CloudI>();
    terrainCloudDwz = std::make_shared<CloudI>();
    plannerCloud = std::make_shared<CloudI>();
    plannerCloudCrop = std::make_shared<CloudI>();
    boundaryCloud = std::make_shared<CloudI>();
    addedObstacles = std::make_shared<CloudI>();

    for (auto &cloud : laserCloudStack) {
      cloud = std::make_shared<CloudI>();
    }

    for (auto &cloud : startPaths) {
      cloud = std::make_shared<Cloud>();
    }

#if PLOTPATHSET == 1
    for (auto &cloud : paths) {
      cloud = std::make_shared<CloudI>();
    }
    freePaths = std::make_shared<CloudI>();
#endif

    for (auto &corr : correspondences) {
      corr.clear();
    }
  }

  void configureFilters()
  {
    laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);
  }

  void setupPublishers()
  {
    pubSlowDown = node_->create_publisher<std_msgs::msg::Int8>("/slow_down", 5);
    pubStop = node_->create_publisher<std_msgs::msg::Int8>("/stop", 5);
    pubPath = node_->create_publisher<nav_msgs::msg::Path>("/path", 5);
#if PLOTPATHSET == 1
    pubFreePaths = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
#endif
    navStatusPub_ = node_->create_publisher<std_msgs::msg::Int32>("/nav_status", 5);
    goalStatusPub_ = node_->create_publisher<autonomy_msgs::msg::GoalStatus>("/goal_status", 5);
    pubCurrentGoal = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/current_goal", 5);
  }

  void setupSubscriptions()
  {
    subOdometry = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 5, std::bind(&LocalPlannerNode::odometryHandler, this, _1));

    subTerrainCloud = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/terrain_map", 5, std::bind(&LocalPlannerNode::terrainCloudHandler, this, _1));

    subRobotCommand = node_->create_subscription<joy_command_mapper::msg::RobotCommand>(
      "/robot_command", 5, std::bind(&LocalPlannerNode::robotCommandHandler, this, _1));

    subNavigateCmd = node_->create_subscription<autonomy_msgs::msg::NavigateCmd>(
      "/nav_cmd", 5, std::bind(&LocalPlannerNode::navigateCmdHandler, this, _1));

    subGoal = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point", 5, std::bind(&LocalPlannerNode::goalHandler, this, _1));

    subSpeed = node_->create_subscription<std_msgs::msg::Float32>(
      "/speed", 5, std::bind(&LocalPlannerNode::speedHandler, this, _1));

    subBoundary = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/navigation_boundary", 5, std::bind(&LocalPlannerNode::boundaryHandler, this, _1));

    subAddedObstacles = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/added_obstacles", 5, std::bind(&LocalPlannerNode::addedObstaclesHandler, this, _1));

    subCheckObstacle = node_->create_subscription<std_msgs::msg::Bool>(
      "/check_obstacle", 5, std::bind(&LocalPlannerNode::checkObstacleHandler, this, _1));
  }

  void setupServices()
  {
    navStatusService_ = node_->create_service<std_srvs::srv::Trigger>(
      "/nav_status/query",
      std::bind(&LocalPlannerNode::handleNavStatusQuery, this, _1, _2));
  }

  void readStaticData()
  {
    RCLCPP_INFO(node_->get_logger(), "Reading path files.");
    readStartPaths();
#if PLOTPATHSET == 1
    readPaths();
#endif
    readPathList();
    readCorrespondences();
  }

  void initializeAutonomyState()
  {
    if (autonomyMode) {
      joySpeed = autonomySpeed / maxSpeed;

      if (joySpeed < 0) joySpeed = 0;
      else if (joySpeed > 1.0) joySpeed = 1.0;
    }
  }

  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    odomTime = rclcpp::Time(odom->header.stamp).seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
  }

  void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
  {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || (point.intensity > groundHeightThre && useCost))) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }

  void robotCommandHandler(const joy_command_mapper::msg::RobotCommand::ConstSharedPtr cmd)
  {
    joyTime = rclcpp::Time(cmd->header.stamp).seconds();
    joySpeedRaw = sqrt(cmd->vy * cmd->vy + cmd->vx * cmd->vx);
    joySpeed = joySpeedRaw;
    if (joySpeed > 1.0) joySpeed = 1.0;
    if (cmd->vx == 0) joySpeed = 0;

    if (joySpeed > 0) {
      joyDir = atan2(cmd->vy, cmd->vx) * 180 / PI;
      if (cmd->vx < 0) joyDir *= -1;
    }

    if (cmd->vx < 0 && !twoWayDrive) joySpeed = 0;

    autonomyMode = cmd->autonomy_mode;
    checkObstacle = cmd->obstacle_check_enabled;
    updateNavStatus(lastGoalReached_, lastGoalStuck_);
  }

  void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
  {
    goalWorld.x = goal->point.x;
    goalWorld.y = goal->point.y;
    goalWorld.z = 0.0f;
    goalYawValid = false;

    goalStopLatched_ = false;
    goalReachedLatched_ = false;

    twoWayDrive = false;
  }

  void navigateCmdHandler(const autonomy_msgs::msg::NavigateCmd::ConstSharedPtr nav_cmd)
  {
    // Extract goal pose from the PoseStamped message
    goalWorld.x = nav_cmd->goal_pose.pose.position.x;
    goalWorld.y = nav_cmd->goal_pose.pose.position.y;
    goalWorld.z = 0.0f;
    const geometry_msgs::msg::Quaternion &orientation = nav_cmd->goal_pose.pose.orientation;
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    if (nav_cmd->track_yaw) {
      quat.normalize();
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      goalYaw = yaw;
      goalYawValid = true;
    } else {
      goalYawValid = false;
    }

    goalStopLatched_ = false;
    goalReachedLatched_ = false;

    // Set autonomy mode from the message
    autonomyMode = nav_cmd->is_autonomy;
    twoWayDrive = nav_cmd->allow_reverse;
    checkObstacle = nav_cmd->check_obstacle;

    joyTime = rclcpp::Time(nav_cmd->goal_pose.header.stamp).seconds();
    joySpeed = 1.0;
    joyDir = 0.0;

    // Publish current goal for visualization
    pubCurrentGoal->publish(nav_cmd->goal_pose);

    RCLCPP_INFO(node_->get_logger(),
                "NavigateCmd received: goal=(%.2f, %.2f), autonomy=%d, twoWayDrive=%d, track_yaw=%d",
                goalWorld.x, goalWorld.y, autonomyMode, twoWayDrive, nav_cmd->track_yaw);
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

  void boundaryHandler(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr boundary)
  {
    boundaryCloud->clear();
    pcl::PointXYZI point, point1, point2;
    int boundarySize = boundary->polygon.points.size();

    if (boundarySize >= 1) {
      point2.x = boundary->polygon.points[0].x;
      point2.y = boundary->polygon.points[0].y;
      point2.z = boundary->polygon.points[0].z;
    }

    for (int i = 0; i < boundarySize; i++) {
      point1 = point2;

      point2.x = boundary->polygon.points[i].x;
      point2.y = boundary->polygon.points[i].y;
      point2.z = boundary->polygon.points[i].z;

      if (point1.z == point2.z) {
        float disX = point1.x - point2.x;
        float disY = point1.y - point2.y;
        float dis = sqrt(disX * disX + disY * disY);

        int pointNum = int(dis / terrainVoxelSize) + 1;
        for (int pointID = 0; pointID < pointNum; pointID++) {
          point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
          point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
          point.z = 0;
          point.intensity = 100.0;

          for (int j = 0; j < pointPerPathThre; j++) {
            boundaryCloud->push_back(point);
    }
  }
}
    }
  }

  void addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr addedObstacles2)
  {
    addedObstacles->clear();
    pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

    int addedObstaclesSize = addedObstacles->points.size();
    for (int i = 0; i < addedObstaclesSize; i++) {
      addedObstacles->points[i].intensity = 200.0;
    }
  }

  void checkObstacleHandler(const std_msgs::msg::Bool::ConstSharedPtr checkObs)
  {
    double checkObsTime = node_->now().seconds();
    if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
      checkObstacle = checkObs->data;
    }
  }

  void handleNavStatusQuery(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
    response->message = std::to_string(navStatus_);
  }

  void publishGoalStatus(bool goalReached, double goalDistance, bool orientationAligned, double yawError)
  {
    if (goalStatusPub_) {
      goalStatusMsg_.header.stamp = node_->now();
      goalStatusMsg_.header.frame_id = "vehicle";
      goalStatusMsg_.goal_reached = goalReached;
      goalStatusMsg_.goal_distance = static_cast<float>(goalDistance);
      goalStatusMsg_.goal_clear_range = static_cast<float>(goalClearRange);
      goalStatusMsg_.orientation_aligned = orientationAligned;
      goalStatusMsg_.yaw_error = static_cast<float>(yawError);
      goalStatusMsg_.autonomy_mode = autonomyMode;
      goalStatusPub_->publish(goalStatusMsg_);
    }
  }

  void updateNavStatus(bool goalReached, bool goalStuck)
  {
    int32_t status = 0;
    if (autonomyMode) {
      status |= NAV_STATUS_ACTIVE;
    } else {
      status |= NAV_STATUS_MANUAL;
    }

    if (goalReached) {
      status |= NAV_STATUS_GOAL_REACHED;
    }
    if (goalStuck) {
      status |= NAV_STATUS_GOAL_STUCK;
    }

    navStatus_ = status;
    lastGoalReached_ = goalReached;
    lastGoalStuck_ = goalStuck;
    navStatusMsg_.data = status;
    if (navStatusPub_) {
      navStatusPub_->publish(navStatusMsg_);
    }
  }

  int readPlyHeader(FILE *filePtr)
  {
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header") {
      val = fscanf(filePtr, "%s", str);
      if (val != 1) {
        RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      strLast = strCur;
      strCur = string(str);

      if (strCur == "vertex" && strLast == "element") {
        val = fscanf(filePtr, "%d", &pointNum);
        if (val != 1) {
          RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
          exit(1);
        }
      }
    }

    return pointNum;
  }

  void readStartPaths()
  {
    string fileName = pathFolder + "/startPaths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(node_->get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++) {
      val1 = fscanf(filePtr, "%f", &point.x);
      val2 = fscanf(filePtr, "%f", &point.y);
      val3 = fscanf(filePtr, "%f", &point.z);
      val4 = fscanf(filePtr, "%d", &groupID);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
        RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      if (groupID >= 0 && groupID < groupNum) {
        startPaths[groupID]->push_back(point);
      }
    }

    fclose(filePtr);
  }

#if PLOTPATHSET == 1
  void readPaths()
  {
    string fileName = pathFolder + "/paths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(node_->get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++) {
      val1 = fscanf(filePtr, "%f", &point.x);
      val2 = fscanf(filePtr, "%f", &point.y);
      val3 = fscanf(filePtr, "%f", &point.z);
      val4 = fscanf(filePtr, "%d", &pathID);
      val5 = fscanf(filePtr, "%f", &point.intensity);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      if (pathID >= 0 && pathID < pathNum) {
        pointSkipCount++;
        if (pointSkipCount > pointSkipNum) {
          paths[pathID]->push_back(point);
          pointSkipCount = 0;
        }
      }
    }

    fclose(filePtr);
  }
#endif

  void readPathList()
  {
    string fileName = pathFolder + "/pathList.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(node_->get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    if (pathNum != readPlyHeader(filePtr)) {
      RCLCPP_INFO(node_->get_logger(), "Incorrect path number, exit.");
      exit(1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    for (int i = 0; i < pathNum; i++) {
      val1 = fscanf(filePtr, "%f", &endX);
      val2 = fscanf(filePtr, "%f", &endY);
      val3 = fscanf(filePtr, "%f", &endZ);
      val4 = fscanf(filePtr, "%d", &pathID);
      val5 = fscanf(filePtr, "%d", &groupID);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
        pathList[pathID] = groupID;
        endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
      }
    }

    fclose(filePtr);
  }

  void readCorrespondences()
  {
    string fileName = pathFolder + "/correspondences.txt";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(node_->get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum; i++) {
      val1 = fscanf(filePtr, "%d", &gridVoxelID);
      if (val1 != 1) {
        RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      while (1) {
        val1 = fscanf(filePtr, "%d", &pathID);
        if (val1 != 1) {
          RCLCPP_INFO(node_->get_logger(), "Error reading input files, exit.");
          exit(1);
        }

        if (pathID != -1) {
          if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
            correspondences[gridVoxelID].push_back(pathID);
          }
        } else {
          break;
        }
      }
    }

    fclose(filePtr);
  }

  void processPlannerCycle()
  {
    if (!(newLaserCloud || newTerrainCloud)) {
      return;
    }

    // // DEBUG: print planner status
    // RCLCPP_INFO(node_->get_logger(), "Planner status: newLaserCloud=%d, newTerrainCloud=%d, goalYawValid=%d",
    //             newLaserCloud, newTerrainCloud, goalYawValid);
    // RCLCPP_INFO(node_->get_logger(), "Planner status: autonomyMode=%d, twoWayDrive=%d, checkObstacle=%d, joySpeed=%.2f, joyDir=%.2f",
    //             autonomyMode, twoWayDrive, checkObstacle, joySpeed, joyDir);

    if (newLaserCloud) {
      newLaserCloud = false;

      laserCloudStack[laserCloudCount]->clear();
      *laserCloudStack[laserCloudCount] = *laserCloudDwz;
      laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

      plannerCloud->clear();
      for (const auto &stackCloud : laserCloudStack) {
        *plannerCloud += *stackCloud;
      }
    }

    if (newTerrainCloud) {
      newTerrainCloud = false;

      plannerCloud->clear();
      *plannerCloud = *terrainCloudDwz;
    }

    const Eigen::Affine3f vehiclePose = pcl::getTransformation(
      vehicleX, vehicleY, vehicleZ, 0.0f, 0.0f, vehicleYaw);
    const Eigen::Affine3f worldToVehicle = vehiclePose.inverse();

    CloudI plannerCloudVehicle;
    plannerCloudVehicle.points.reserve(plannerCloud->points.size());
    pcl::transformPointCloud(*plannerCloud, plannerCloudVehicle, worldToVehicle);

    plannerCloudCrop->clear();
    plannerCloudCrop->points.reserve(
      plannerCloudVehicle.points.size() + boundaryCloud->points.size() + addedObstacles->points.size());
    plannerCloudCrop->points.insert(
      plannerCloudCrop->points.end(),
      plannerCloudVehicle.points.begin(),
      plannerCloudVehicle.points.end());
    
    if (boundaryCloud->points.size() > 0) {
      CloudI boundaryCloudVehicle;
      boundaryCloudVehicle.points.reserve(boundaryCloud->points.size());
      pcl::transformPointCloud(*boundaryCloud, boundaryCloudVehicle, worldToVehicle);
      for (const auto &transformedPoint : boundaryCloudVehicle.points) {
        float dis = sqrt(transformedPoint.x * transformedPoint.x + transformedPoint.y * transformedPoint.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->points.push_back(transformedPoint);
        }
      }
    }

    if (addedObstacles->points.size() > 0) {
      CloudI addedObstaclesVehicle;
      addedObstaclesVehicle.points.reserve(addedObstacles->points.size());
      pcl::transformPointCloud(*addedObstacles, addedObstaclesVehicle, worldToVehicle);
      for (const auto &transformedPoint : addedObstaclesVehicle.points) {
        float dis = sqrt(transformedPoint.x * transformedPoint.x + transformedPoint.y * transformedPoint.y);
        if (dis < adjacentRange) {
            plannerCloudCrop->points.push_back(transformedPoint);
          }
        }
    }

    const auto croppedPointCount = plannerCloudCrop->points.size();
    plannerCloudCrop->width = croppedPointCount;
    plannerCloudCrop->height = croppedPointCount > 0 ? 1 : 0;
    plannerCloudCrop->is_dense = croppedPointCount > 0 ? false : true;

    float pathRange = adjacentRange;
    float goalDistanceForStatus = adjacentRange;
    if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
    if (pathRange < minPathRange) pathRange = minPathRange;
    float relativeGoalDis = adjacentRange;

    /************************** Handle the freeze state **************************/
    /** Freeze the vehilce to only rotate if goal isn't within the freeze range */
    if (autonomyMode) {
      pcl::PointXYZ goalVehicle = pcl::transformPoint(goalWorld, worldToVehicle);
      relativeGoalDis = sqrt(goalVehicle.x * goalVehicle.x + goalVehicle.y * goalVehicle.y);
      goalDistanceForStatus = relativeGoalDis;
      joyDir = atan2(goalVehicle.y, goalVehicle.x) * 180 / PI;

      if (fabs(joyDir) > freezeAng && relativeGoalDis < goalBehindRange) {
        relativeGoalDis = 0;
        joyDir = 0;
      }

      if (fabs(joyDir) > freezeAng && freezeStatus == 0) {
        freezeStartTime = odomTime;
        freezeStatus = 1;
      } else if (odomTime - freezeStartTime > freezeTime && freezeStatus == 1) {
        freezeStatus = 2;
      } else if (fabs(joyDir) <= freezeAng && freezeStatus == 2) {
        freezeStatus = 0;
      }

      if (!twoWayDrive) {
        if (joyDir > 95.0) joyDir = 95.0;
        else if (joyDir < -95.0) joyDir = -95.0;
      }
    } else {
      freezeStatus = 0;
    }

    if (freezeStatus == 1 && autonomyMode) {
      relativeGoalDis = 0;
      joyDir = 0;
    }

    /************************** Plan the path **************************/

    std::array<int, rotationSampleCount * pathNum> clearPathList{};
    std::array<float, rotationSampleCount * pathNum> pathPenaltyList{};
    std::array<float, rotationSampleCount * groupNum> clearPathPerGroupScore{};
    std::array<int, rotationSampleCount * groupNum> clearPathPerGroupNum{};
    std::array<float, rotationSampleCount * groupNum> pathPenaltyPerGroupScore{};

    bool hasValidPath = false;
    bool orientationAligned = true;
    double yawError = 0.0;

    // Apply hysteresis to goalClearRange to prevent oscillation
    // When outside range: need to get within goalClearRange to activate
    // When inside range: need to exceed goalClearRange * hysteresis to deactivate
    double effectiveGoalClearRange = goalClearRange;
    if (goalWithinClearRange_) {
      // Currently within range - use expanded range for deactivation
      effectiveGoalClearRange = goalClearRange * goalClearRangeHysteresis;
    }

    // Update state: are we currently within the clear range?
    if (!goalWithinClearRange_ && goalDistanceForStatus <= goalClearRange) {
      // Entering the clear range
      goalWithinClearRange_ = true;
    } else if (goalWithinClearRange_ && goalDistanceForStatus > goalClearRange * goalClearRangeHysteresis) {
      // Exiting the clear range (with hysteresis)
      goalWithinClearRange_ = false;
    }

    if (autonomyMode && goalYawValid) {
      yawError = std::atan2(std::sin(goalYaw - vehicleYaw), std::cos(goalYaw - vehicleYaw));
      orientationAligned = fabs(yawError) <= goalYawToleranceRad;
    }
    bool goalReached = autonomyMode && goalWithinClearRange_ && orientationAligned;
    // RCLCPP_INFO(node_->get_logger(), "Goal reached: %d, goalDistanceForStatus: %.2f, effectiveGoalClearRange: %.2f, goalWithinClearRange: %d, orientationAligned: %d", goalReached, goalDistanceForStatus, effectiveGoalClearRange, goalWithinClearRange_, orientationAligned);

    if (goalReached) {
      goalReachedLatched_ = true;
    }

    // Publish detailed goal status for RViz plugin
    publishGoalStatus(goalReachedLatched_, goalDistanceForStatus, orientationAligned, yawError);

    // Publish stop command to pathFollower when goal is reached
    publishStopCommand(goalReached);

    float defPathScale = pathScale;
    // Snapshot the configured path scale so we can restore it after any per-cycle adjustments.
    if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
    // Clamp the dynamic scale so it never shrinks below the minimum allowed value.
    if (pathScale < minPathScale) pathScale = minPathScale;

    // Iteratively test the path library, shrinking scale/range only when no valid option is found.
    while (pathScale >= minPathScale && pathRange >= minPathRange) {
      resetPathStatistics(
        clearPathList, pathPenaltyList,
        clearPathPerGroupScore, clearPathPerGroupNum, pathPenaltyPerGroupScore);

      float minObsAngCW = -180.0;
      float minObsAngCCW = 180.0;
      gatherObstacleStatistics(
        pathScale, pathRange, relativeGoalDis, joyDir,
        minObsAngCW, minObsAngCCW, clearPathList, pathPenaltyList);

      float penaltyScore = 0;
      int selectedGroupID = evaluatePathScores(
        relativeGoalDis, joyDir, minObsAngCW, minObsAngCCW, penaltyScore,
        clearPathList, pathPenaltyList,
        clearPathPerGroupScore, clearPathPerGroupNum, pathPenaltyPerGroupScore);

      publishSlowDownCommand(penaltyScore);

      if (selectedGroupID >= 0) {
        if (publishSelectedPath(
            selectedGroupID, pathScale, pathRange, relativeGoalDis, joyDir,
            minObsAngCW, minObsAngCCW, clearPathList)) {
          hasValidPath = true;
        }
      }

      if (hasValidPath) {
        break;
      }

      // Reduce the template size first, then shorten the look-ahead range if scaling can no longer shrink.
      if (pathScale >= minPathScale + pathScaleStep) {
        pathScale -= pathScaleStep;
        pathRange = adjacentRange * pathScale / defPathScale;
      } else {
        pathRange -= pathRangeStep;
      }
    }
    // Restore the original configured scale before starting the next planning cycle.
    pathScale = defPathScale;

    if (!hasValidPath) {
      pathMsg_.poses.resize(1);
      pathMsg_.poses[0].pose.position.x = 0;
      pathMsg_.poses[0].pose.position.y = 0;
      pathMsg_.poses[0].pose.position.z = 0;

      pathMsg_.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      pathMsg_.header.frame_id = "vehicle";
      pubPath->publish(pathMsg_);

#if PLOTPATHSET == 1
      freePaths->clear();
      sensor_msgs::msg::PointCloud2 freePaths2;
      pcl::toROSMsg(*freePaths, freePaths2);
      freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      freePaths2.header.frame_id = "vehicle";
      pubFreePaths->publish(freePaths2);
#endif
    }

    if (autonomyMode) {
      if (hasValidPath) {
        noPathCyclesWithoutPath_ = 0;
      } else if (noPathCyclesWithoutPath_ < noPathCycleThreshold) {
        ++noPathCyclesWithoutPath_;
      }
    } else {
      noPathCyclesWithoutPath_ = 0;
    }

    bool goalStuck = autonomyMode && (noPathCyclesWithoutPath_ >= noPathCycleThreshold);
    updateNavStatus(goalReachedLatched_, goalStuck);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("localPlanner");
  auto planner = std::make_shared<LocalPlannerNode>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
