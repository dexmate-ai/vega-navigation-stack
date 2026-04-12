# Detailed goal status message from local planner
# Contains information about goal reached conditions

# Header for timestamp
std_msgs/Header header

# Goal reached flag (true when goal is reached)
bool goal_reached

# Distance from current position to goal (meters)
float32 goal_distance

# Distance threshold for considering goal reached (meters)
float32 goal_clear_range

# Whether orientation is aligned with goal (when track_yaw is enabled)
bool orientation_aligned

# Yaw error in radians (only valid when track_yaw is enabled)
float32 yaw_error

# Whether the planner is in autonomy mode
bool autonomy_mode
