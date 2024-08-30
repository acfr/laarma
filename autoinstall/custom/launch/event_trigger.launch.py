from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_arguments = []

    params_file = LaunchConfiguration("params_file", default="/mnt/config/detection_trigger_params.yaml")
    near_params_file = LaunchConfiguration("params_file", default="/mnt/config/detection_trigger_near_params.yaml")
    zoom_params_file = LaunchConfiguration("params_file", default="/mnt/config/detection_trigger_zoom_params.yaml")
    zoom2_params_file = LaunchConfiguration("params_file", default="/mnt/config/detection_trigger_zoom2_params.yaml")
    far_params_file = LaunchConfiguration("params_file", default="/mnt/config/detection_trigger_far_params.yaml")
    log_level = LaunchConfiguration("log_level", default="info")

    return LaunchDescription(
        [
            *launch_arguments,
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="detection_trigger_cam_near",
                parameters=[
                    near_params_file,
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
                name="detection_trigger_cam_near_zoomed",
                parameters=[
                    zoom_params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_near/zoomed/resized/detection"),
                    ("/out/trigger", "/detection_trigger/camera_near_zoomed"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # new zoom
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="detection_trigger_cam_near_zoomed2",
                parameters=[
                    zoom2_params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_near/zoomed2/resized/detection"),
                    ("/out/trigger", "/detection_trigger/camera_near_zoomed2"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="detection_trigger_cam_far",
                parameters=[
                    far_params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_far/resized/detection"),
                    ("/out/trigger", "/detection_trigger/camera_far"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="event_trigger",
                executable="detection_trigger",
                name="detection_trigger_cam_thermal",
                parameters=[
                    params_file,
                ],
                remappings=[
                    ("/in/detection", "/camera_thermal/detection"),
                    ("/out/trigger", "/detection_trigger/camera_thermal"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="event_trigger",
                executable="event_trigger",
                name="event_trigger",
                parameters=[
                    {"use_system_default_qos": True},
                    {"event_signal_duration": 57},
                    {"event_config_file": "/mnt/config/event_trigger_config.yaml"},
                ],
                remappings=[
                    ("/out/event", "/event_trigger/event"),
                    ("/out/event_signal", "/event_trigger/event_signal"),
                ],
                output="both",
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
