# ObjectInfo.msg — object information published to /target_pose
std_msgs/Header header

# Object class identifier
string object_class

# Bottom center position (object bottom surface center)
geometry_msgs/Point bottom_center

# 3D bounding box dimensions (x, y, z) in meters
float32[3] bbox_size

# Detection confidence [0.0, 1.0]
float32 confidence
