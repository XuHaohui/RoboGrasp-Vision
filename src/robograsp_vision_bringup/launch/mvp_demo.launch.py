
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # -- Static TF: world -> camera_depth_optical_frame (camera at 0.4, 0, 1.2, looking down) --
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_camera",
        output="screen",
        arguments=["0.4", "0", "1.2", "1", "0", "0", "0", "world", "camera_depth_optical_frame"],
    )

    # -- Perception node --
    perception_node = Node(
        package="robograsp_vision_perception",
        executable="perception_node",
        name="perception",
        output="screen",
        parameters=[{
            "sensor_frame": "camera_depth_optical_frame",
        }],
    )

    # -- Robot Bridge node --
    bridge_node = Node(
        package="robograsp_vision_bridge",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
        parameters=[{
            "robot_backend": "piper_control",
            "target_topic": "/target_pose",
            "target_frame": "world",
        }],
    )

    ld.add_action(static_tf)
    ld.add_action(perception_node)
    ld.add_action(bridge_node)

    return ld
