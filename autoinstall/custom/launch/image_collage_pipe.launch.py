from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_arguments = []

    log_level = LaunchConfiguration("log_level", default="info")

    collage_params = LaunchConfiguration("collage_params", default="/mnt/config/img_collage_params.yaml")
    collage_throttle_params = LaunchConfiguration(
        "collage_throttle_params", default="/mnt/config/img_collage_throttle_params.yaml"
    )

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
                    collage_params,
                ],
                remappings=[
                    ("/in/image0", "/camera_near/resized/image/detection"),
                    ("/in/image1", "/camera_near/zoomed/resized/image/detection"),
                    ("/in/image2", "/camera_far/resized/image/detection"),
                    ("/in/image3", "/camera_thermal/image/detection"),
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
                    collage_throttle_params,
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
