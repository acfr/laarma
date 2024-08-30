import os

from launch import LaunchDescription
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_arguments = []

    context = LaunchContext()
    default_params_file = os.path.join(
        FindPackageShare("event_trigger").perform(context), "config/detection_trigger_params.yaml"
    )

    params_file = LaunchConfiguration("params_file", default=str(default_params_file))
    log_level = LaunchConfiguration("log_level", default="info")

    return LaunchDescription(
        [
            *launch_arguments,
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="example_detection_trigger1",
                parameters=[
                    params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_near/resized/detection"),
                    ("/out/trigger", "/detection_trigger/camera_near"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="example_detection_trigger2",
                parameters=[
                    params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_near/zoomed/resized/detection"),
                    ("/out/trigger", "/detection_trigger/camera_near_zoomed"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="example_detection_trigger3",
                parameters=[
                    params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_far/resized/detection"),
                    ("/out/trigger", "/detection_trigger/camera_far"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
