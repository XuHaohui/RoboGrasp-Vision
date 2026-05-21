"""MVP demo launch: perception + bridge pipeline in mock mode (keyboard control).

Data flow:
    keyboard -> mock_perception -> /perception/result (camera_link)
    -> bridge_node -> tf2 transform -> /target_pose (world)

Usage:
    ros2 launch robograsp_vision_bringup mvp_demo.launch.py

Requirements:
    - ROS2 Humble sourced
    - Workspace built (colcon build)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # -- Static TF: world -> camera_link (identity) --
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_camera",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "camera_link"],
    )

    # -- Mock Perception node (keyboard mode) --
    mock_perception_node = Node(
        package="robograsp_vision_perception",
        executable="mock_perception",
        name="mock_perception",
        output="screen",
        prefix="xterm -e",
        parameters=[{
            "sensor_frame": "camera_link",
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
    ld.add_action(mock_perception_node)
    ld.add_action(bridge_node)

    return ld
