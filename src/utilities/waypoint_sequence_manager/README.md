# Waypoint Sequence Manager

This package provides waypoint sequence management for factory environments where robots need to navigate through predefined waypoint sequences (e.g., point1->point2->point3->point1->...).

## Features

- **Automatic Waypoint Sequencing**: Loops through waypoint sequences continuously
- **Position Tracking**: Monitors robot position and automatically advances to next waypoint when current one is reached
- **Control Commands**: Start, stop, pause, resume, reset, and manual advance functionality
- **Visual Feedback**: Publishes visualization markers for RViz display
- **Status Reporting**: Real-time status updates with sequence progress

## Topics

### Published Topics
- `/way_point` (geometry_msgs/PointStamped): Current target waypoint
- `/waypoint_sequence_status` (std_msgs/String): Status updates in format "STATUS|total_waypoints|current_index|sequence_id"
- `/waypoint_sequence_markers` (visualization_msgs/MarkerArray): Visualization markers for RViz

### Subscribed Topics
- `/state_estimation` (nav_msgs/Odometry): Robot position for waypoint reached detection
- `/new_waypoint_sequence` (std_msgs/String): JSON-formatted waypoint sequences
- `/waypoint_sequence_control` (std_msgs/String): Control commands

## Control Commands

- `START`: Begin waypoint sequence execution
- `STOP`: Stop sequence execution
- `PAUSE`: Pause current sequence
- `RESUME`: Resume paused sequence
- `RESET`: Reset to first waypoint
- `NEXT`: Manually advance to next waypoint

## Waypoint Sequence Format

Waypoints are sent as JSON strings:

```json
{
  "waypoints": [
    {"x": 2.0, "y": 1.0, "z": 0.0},
    {"x": 5.0, "y": 1.0, "z": 0.0},
    {"x": 5.0, "y": 4.0, "z": 0.0}
  ]
}
```

## Usage

### Launch the Manager
```bash
ros2 launch waypoint_sequence_manager waypoint_sequence_manager.launch.py
```

### Send Waypoints Programmatically
```python
import json
from std_msgs.msg import String

# Create waypoint sequence
waypoints = [
    {"x": 2.0, "y": 1.0, "z": 0.0},
    {"x": 5.0, "y": 1.0, "z": 0.0}
]
sequence_data = {"waypoints": waypoints}

# Publish to topic
msg = String()
msg.data = json.dumps(sequence_data)
sequence_publisher.publish(msg)
```

### Control Sequence
```bash
# Start sequence
ros2 topic pub --once /waypoint_sequence_control std_msgs/String "data: 'START'"

# Stop sequence
ros2 topic pub --once /waypoint_sequence_control std_msgs/String "data: 'STOP'"
```

## Parameters

- `goal_tolerance` (default: 0.5): Distance tolerance in meters to consider waypoint reached
- `sequence_topic` (default: '/waypoint_sequence'): Topic for waypoint sequence data
- `new_sequence_topic` (default: '/new_waypoint_sequence'): Topic for receiving new sequences
- `sequence_control_topic` (default: '/waypoint_sequence_control'): Topic for control commands

## Integration

This package works with the `waypoint_sequence_rviz_plugin` for GUI-based waypoint sequence creation and control through RViz.