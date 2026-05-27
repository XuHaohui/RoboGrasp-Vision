
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # -- Launch arguments --
    ld.add_action(DeclareLaunchArgument(
        "target_topic", default_value="/target_pose",
        description="Target topic for object info"
    ))
    ld.add_action(DeclareLaunchArgument(
        "target_frame", default_value="world",
        description="Target frame for output coordinates"
    ))

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

    # -- Bridge node --
    bridge_node = Node(
        package="robograsp_vision_bridge",
        executable="bridge_node",
        name="bridge",
        output="screen",
        parameters=[{
            "target_topic": LaunchConfiguration("target_topic"),
            "target_frame": LaunchConfiguration("target_frame"),
        }],
    )

    ld.add_action(static_tf)
    ld.add_action(perception_node)

    ld.add_action(
        TimerAction(period=1.0, actions=[bridge_node])
    )

    return ld
