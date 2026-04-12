# Waypoint Sequence RViz Plugin

This RViz plugin provides a graphical interface for creating and controlling waypoint sequences that work with the `waypoint_sequence_manager` package.

## Features

- **Intuitive GUI**: Easy-to-use panel in RViz for waypoint sequence management
- **Waypoint Creation**: Add waypoints by specifying coordinates or robot's current position
- **Sequence Control**: Start, stop, pause, resume, reset, and manually advance through sequences
- **Visual Feedback**: Real-time status display showing current sequence state
- **Integration**: Works seamlessly with `waypoint_sequence_manager`

## Installation

1. Build the package:
```bash
colcon build --packages-select waypoint_sequence_rviz_plugin
```

2. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Adding the Plugin to RViz

1. Launch RViz:
```bash
ros2 run rviz2 rviz2
```

2. Add the plugin:
   - Go to `Panels` → `Add New Panel`
   - Select `waypoint_sequence_rviz_plugin/WaypointSequencePanel`
   - The panel will appear in the RViz interface

### Creating Waypoint Sequences

1. **Start New Sequence**:
   - Click "New Sequence" button
   - This clears any existing waypoints and enables waypoint addition

2. **Add Waypoints**:
   - Click "Add Current Position" to add the robot's current position
   - Or manually enter coordinates when prompted
   - Waypoints are displayed in the text area

3. **Finish Sequence**:
   - Click "Finish Sequence" when done adding waypoints
   - The sequence is automatically sent to the waypoint manager

### Controlling Sequences

- **Start**: Begin executing the waypoint sequence
- **Stop**: Stop sequence execution
- **Pause**: Temporarily pause the current sequence
- **Resume**: Continue from where the sequence was paused
- **Reset**: Return to the first waypoint
- **Next Waypoint**: Manually advance to the next waypoint

### Status Display

The status panel shows:
- Current sequence status (IDLE, ACTIVE, PAUSED, etc.)
- Progress through sequence (current/total waypoints)
- Sequence ID for tracking different sequences

## Panel Sections

### Waypoint Sequence Creation
- New Sequence: Start creating a new waypoint sequence
- Add Current Position: Add robot's current position as waypoint
- Finish Sequence: Complete and send the sequence
- Clear Waypoints: Remove all waypoints from current sequence
- Waypoint Display: Shows list of added waypoints

### Sequence Control
- Control buttons for managing sequence execution
- Real-time control of active sequences

### Status
- Live status updates from the waypoint sequence manager
- Progress tracking and sequence information

## Integration with Autonomy Stack

This plugin is designed to work with the existing autonomy stack:
- Publishes to `/way_point` topic that the local planner subscribes to
- Integrates with `/state_estimation` for position tracking
- Compatible with existing navigation and collision avoidance systems

## Example Workflow

1. Launch the waypoint sequence manager
2. Open RViz with the waypoint sequence panel
3. Create a new sequence and add factory station waypoints
4. Finish the sequence to send it to the manager
5. Use Start to begin autonomous navigation through the sequence
6. The robot will continuously loop through the waypoints: Station1 → Station2 → Station3 → Station1...

## Topics Used

- `/new_waypoint_sequence`: Sends new waypoint sequences to manager
- `/waypoint_sequence_control`: Sends control commands
- `/waypoint_sequence_status`: Receives status updates from manager