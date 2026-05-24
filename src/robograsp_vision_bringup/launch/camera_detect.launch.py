
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_camera",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "camera_link"],
    )

    perception_node = Node(
        package="robograsp_vision_perception",
        executable="perception_node",
        name="perception",
        output="screen",
        parameters=[{
            "use_mock": False,
            "auto_publish": False,
            "sensor_frame": "camera_link",
            "publish_rate": 5.0,
        }],
    )

    bridge_node = Node(
        package="robograsp_vision_bridge",
        executable="bridge_node",
        name="bridge",
        output="screen",
        parameters=[{
            "robot_backend": "piper_control",
            "target_topic": "/target_pose",
            "target_frame": "world",
        }],
    )

    ld.add_action(static_tf)
    ld.add_action(perception_node)
    ld.add_action(TimerAction(period=1.0, actions=[bridge_node]))

    return ld
