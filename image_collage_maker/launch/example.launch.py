from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_arguments = []

    log_level = LaunchConfiguration("log_level", default="info")

    container = ComposableNodeContainer(
        name="image_collage_maker",
        namespace="image_collage_maker",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_collage_maker",
                plugin="CollageMaker",
                name="image_collage_maker_node",
                parameters=[
                    # collage_params,
                    {"use_wall_clock": True},
                    {"msgs_per_sec": 10.0},
                    {"viz": True},
                ],
                remappings=[
                    ("/in/image0", "/camera_near/image"),
                    ("/in/image1", "/camera_near/zoomed/image"),
                    ("/in/image2", "/camera_far/image"),
                    ("/in/image3", "/camera_thermal/image"),
                    ("/out/image_collage", "/image_collage_maker/image"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="topic_tools",
                plugin="topic_tools::ThrottleNode",
                name="image_collage_maker_throttle_node",
                parameters=[
                    {"input_topic": "/image_collage_maker/image"},
                    {"output_topic": "/image_collage_maker/throttled/image"},
                    # collage_throttle_params,
                    {"throttle_type": "messages"},
                    {"msgs_per_sec": 1.0},
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
