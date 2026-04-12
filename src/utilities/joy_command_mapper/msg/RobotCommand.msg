# Header for timestamp and source identification
std_msgs/Header header

# Velocity commands (simplified)
float32 vx  # forward/backward velocity (m/s)
float32 vy  # left/right velocity (m/s)
float32 wz  # angular velocity (rad/s)

# Mode settings
bool autonomy_mode
bool manual_mode  
bool obstacle_check_enabled

# Action triggers (single frame pulses)
bool clear_cloud
bool emergency_stop

# Controller metadata
string source  # "joystick", "rviz_teleop", "waypoint_tool", etc.
float32 deadman_switch  # 0.0-1.0, confidence/safety factor