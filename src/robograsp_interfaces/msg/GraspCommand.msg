# GraspCommand.msg — grasp execution command sent to the robot bridge
std_msgs/Header header

# Target object identifier
string object_class

# Final grasp pose (end-effector)
geometry_msgs/PoseStamped grasp_pose

# Pre-grasp approach pose
geometry_msgs/PoseStamped pre_grasp_pose

# Gripper opening width (m)
float32 grasp_width

# Approach direction unit vector (x, y, z)
float32[3] approach_direction

# Grasp quality metric [0.0, 1.0]
float32 quality

# Strategy label: "top", "side", "custom"
string strategy

# Execution flags
bool use_cartesian_approach

# Original object position (from perception) — used by bridge as /target_pose input
# piper_control expects the object BOTTOM center, not the grasp pose
geometry_msgs/Point object_position

# Sequence identifier for tracking
uint32 sequence_id
