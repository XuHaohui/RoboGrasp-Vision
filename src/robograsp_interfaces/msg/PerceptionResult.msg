# PerceptionResult.msg — output of the perception/detection layer
std_msgs/Header header

# Detected object identifier
string object_class

# Detection confidence [0.0, 1.0]
float32 confidence

# 3D position of the object in the camera/robot frame
geometry_msgs/Point position

# Oriented bounding box dimensions (optional, set to -1 if unused)
float32[3] bbox_size

# 2D bounding box in image coordinates (x, y, w, h), optional
float32[4] bbox_2d

# Sensor frame in which the detection was made
string sensor_frame

# Status: "detected", "lost", "unknown", "error"
string status
