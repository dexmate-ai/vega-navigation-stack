# SLAM Interface

ROS2 package that serves as a comprehensive interface between SLAM and autonomy stack. This node handles state estimation publishing, pointcloud preprocessing, and static frame management.

## Features

### 1. **State Estimation Publishing** ⭐ NEW
Publishes `/state_estimation` topic (nav_msgs/Odometry) at 100Hz by looking up the TF transform from map to vehicle frame. This topic is consumed by:
- `local_planner` - for navigation decisions
- `sensor_scan_generation` - for scan synchronization  
- `visualization_tools` - for rviz displays
- `waypoint_rviz_plugin` - for interactive waypoint placement

### 2. **Static Transform Management**
Publishes static TF transforms using user-configured calibration:
- `map → init_frame` - Aligns SLAM world frame with robot body frame
- `laser_imu_frame → vehicle` - Defines IMU mounting position relative to robot body
- `laser_imu_frame → laser_frame` - Defines lidar extrinsic calibration relative to IMU

### 3. **Pointcloud Preprocessing**
Filters robot body points from SLAM pointclouds:
- Input: `/registered_scan` (in `init_frame` frame from SLAM)
- Output: `/registered_scan_filtered_map` (in `map` frame, body-filtered)
- Configurable 3D box filter to remove robot body points
- Prevents self-collision in navigation planning

## Architecture

```
SLAM (super_odometry)
    ↓ (publishes /registered_scan in init_frame frame)
    ↓ (publishes TF: init_frame -> laser_imu_frame)
    ↓
slam_interface node:
  • Looks up TF: map -> vehicle
  • Publishes /state_estimation (Odometry)
  • Publishes static TF: map -> init_frame
  • Publishes static TF: laser_imu_frame -> vehicle
  • Publishes static TF: laser_imu_frame -> laser_frame
  • Filters /registered_scan → /registered_scan_filtered_map
    ↓
Autonomy Stack (local_planner, terrain_analysis, etc.)
```

## Configuration

All configuration is done via YAML files. See `config/` directory for examples.

### Transform Configuration (`config/transform_*.yaml`)

Defines the calibration between frames for proper sensor-to-robot alignment:

```yaml
/**:
  ros__parameters:
    # Frame names
    map_frame: "map"
    init_frame: "lidar_init"
    vehicle_frame: "vehicle"
    laser_imu_frame: "imu_link"
    laser_frame: "lidar_3d"

    # T_v_i: Transform from IMU to vehicle frame
    # IMU offset from robot body (meters)
    t_v_i:
      x: 0.0
      y: 0.0
      z: 0.0

    # 3x3 rotation matrix from IMU to vehicle (row-major)
    R_v_i:
      data: [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    
    # T_i_l: Transform from laser to IMU frame (lidar extrinsic)
    # Lidar offset from IMU (meters)
    t_i_l:
      x: 0.0
      y: 0.0
      z: 0.0

    # 3x3 rotation matrix from laser to IMU (row-major)
    R_i_l:
      data: [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
```

**Frame Configuration:**
- `map_frame` and `vehicle_frame` are used for the `/state_estimation` topic
- `T_v_i` (vehicle ← IMU) defines the IMU mounting position and is used for:
  - `map → init_frame` static transform
  - `laser_imu_frame → vehicle` static transform (inverse of T_v_i)
- `T_i_l` (IMU ← laser) defines the lidar extrinsic calibration:
  - `laser_imu_frame → laser_frame` static transform

**Backward Compatibility:**
- Old parameter names `translation` and `rotation_matrix` are still supported
- They map to `t_v_i` and `R_v_i` respectively

### Body Filter Configuration (`config/body_filter_*.yaml`)

Defines the filtering box to remove robot body points:

```yaml
/**:
  ros__parameters:
    filter_enabled: true
    vehicle_frame: "vehicle"
    
    # Box bounds in vehicle frame (points inside are removed)
    box_filter:
      min_x: -0.3
      max_x: 0.0
      min_y: -0.3
      max_y: 0.3
      min_z: 0.0
      max_z: 2.0
    
    input_topic: "/registered_scan"
    output_topic: "/registered_scan_filtered_map"
```

## Usage

### Build
```bash
cd ~/autonomy_stack_vega
colcon build --packages-select slam_interface
source install/setup.bash
```

### Launch
```bash
# With default configs
ros2 launch slam_interface slam_interface.launch.py

# With custom configs
ros2 launch slam_interface slam_interface.launch.py \
  transform_config:=/path/to/my_transform.yaml \
  filter_config:=/path/to/my_filter.yaml
```

### Verify Operation

**Check state estimation:**
```bash
ros2 topic echo /state_estimation
ros2 topic hz /state_estimation  # Should show ~100 Hz
```

**Check transforms:**
```bash
ros2 run tf2_ros tf2_echo map init_frame
ros2 run tf2_ros tf2_echo laser vehicle
ros2 run tf2_ros tf2_echo map vehicle  # This is published by SLAM
```

**Check pointcloud processing:**
```bash
ros2 topic echo /registered_scan_filtered_map
```

## Quick Calibration

If you have a 4×4 transformation matrix from calibration:
```
T = [R11  R12  R13  tx]
    [R21  R22  R23  ty]
    [R31  R32  R33  tz]
    [0    0    0    1 ]
```

Extract and add to your config YAML:
```yaml
translation:
  x: tx
  y: ty  
  z: tz

rotation_matrix:
  data: [R11, R12, R13,
         R21, R22, R23,
         R31, R32, R33]
```

## Frame Hierarchy

```
map (robot body aligned)          vehicle (robot body)
 └─> init_frame (SLAM frame)       └─> laser (sensor)
```

Both static transforms use the same T_body_imu calibration (forward and inverse).

The dynamic transform `init_frame → vehicle` is published by SLAM (super_odometry).

## Topics

### Published
- `/state_estimation` (nav_msgs/Odometry) - Robot pose in map frame at 100Hz
- `/registered_scan_filtered_map` (sensor_msgs/PointCloud2) - Body-filtered pointcloud in map frame

### Subscribed
- `/registered_scan` (sensor_msgs/PointCloud2) - Raw SLAM pointcloud in init_frame frame

### TF Published
- `map → init_frame` (static)
- `laser → vehicle` (static)

### TF Required
- `map → vehicle` (dynamic, looked up for state_estimation)
- `init_frame → vehicle` (dynamic, from SLAM, used for pointcloud filtering)

## Integration with Base Autonomy

This node is a critical dependency for the entire autonomy stack. It replaces the need for:
- Separate `state_estimation_publisher` node
- Separate `pointcloud_preprocessor` node  
- Manual static transform broadcasting

All base_autonomy nodes that previously subscribed to `/state_estimation` will now use this unified interface.

## Example Configs

- `transform_default.yaml` - Identity transform (sensor aligned with body)
- `transform_airy_zm90_xm45.yaml` - RS-Airy sensor calibration
- `transform_mid360_y30.yaml` - Livox MID360 sensor calibration (30° Y-axis rotation)
- `body_filter_default.yaml` - Standard body filter box

## BUG

If slam_interface launches after global mapping succeeds, the tf tree will be problematic. Need a more robust way of handling localization mode.
