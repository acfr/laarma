from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_arguments = []

    lidar_ns = LaunchConfiguration("lidar_ns", default="neuvition_lidar")
    log_level = LaunchConfiguration("log_level", default="info")

    lidar_params = LaunchConfiguration("lidar_params", default="/mnt/config/neuvition_lidar_params.yaml")
    lidar_throttle_params = LaunchConfiguration(
        "lidar_throttle_params", default="/mnt/config/lidar_throttle_params.yaml"
    )

    container = ComposableNodeContainer(
        name=[lidar_ns, "_pipe"],
        namespace=lidar_ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="neuvition_lidar",
                plugin="LidarStreamer",
                name=[lidar_ns, "_driver_node"],
                parameters=[
                    lidar_params,
                    {"lidar_ns": lidar_ns},
                ],
                remappings=[],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="topic_tools",
                plugin="topic_tools::ThrottleNode",
                name=[lidar_ns, "_throttle_node"],
                parameters=[
                    {"input_topic": ["/", lidar_ns, "/points"]},
                    {"output_topic": ["/", lidar_ns, "/throttled/points"]},
                    lidar_throttle_params,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
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
