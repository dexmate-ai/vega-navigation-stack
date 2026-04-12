# Navigation command message for local planner
# Contains goal pose and navigation behavior settings

# Goal pose for navigation
geometry_msgs/PoseStamped goal_pose

# Enable/disable autonomy mode
bool is_autonomy

# Allow reverse motion during navigation
bool allow_reverse

# Track goal yaw orientation (if false, only position matters)
bool track_yaw

bool check_obstacle
