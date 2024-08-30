from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_arguments = []

    log_level = LaunchConfiguration("log_level", default="info")

    return LaunchDescription(
        [
            *launch_arguments,
            Node(
                package="event_trigger",
                executable="event_trigger",
                name="example_event_trigger",
                parameters=[],
                remappings=[
                    ("/out/event", "/event_trigger/event"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
