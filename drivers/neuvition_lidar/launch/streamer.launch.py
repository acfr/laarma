import os

from launch import LaunchDescription
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_arguments = []

    context = LaunchContext()
    default_params_file = os.path.join(FindPackageShare("neuvition_lidar").perform(context), "config/default.yaml")

    params_file = LaunchConfiguration("params_file", default=str(default_params_file))
    log_level = LaunchConfiguration("log_level", default="info")

    container = ComposableNodeContainer(
        name="lidar_streamer",
        namespace="lidar",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="neuvition_lidar",
                plugin="LidarStreamer",
                name="lidar_streamer",
                parameters=[params_file],
                remappings=[],
                extra_arguments=[{"use_intra_process_comms": False}],
            ),
        ],
        output="both",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            *launch_arguments,
            container,
        ]
    )
