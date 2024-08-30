from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    cam_ns = LaunchConfiguration("cam_ns", default="camera_far")
    log_level = LaunchConfiguration("log_level", default="info")

    cam_far_params = LaunchConfiguration("cam_far_params", default="/mnt/config/cam_far_params.yaml")
    img_resize_params = LaunchConfiguration("img_resize_params", default="/mnt/config/img_resize_params.yaml")
    img_throttle_params = LaunchConfiguration("img_throttle_params", default="/mnt/config/img_throttle_params.yaml")
    img_ffmpeg_params = LaunchConfiguration("img_ffmpeg_params", default="/mnt/config/img_ffmpeg_params.yaml")
    img_yolo_params = LaunchConfiguration("img_yolo_params", default="/mnt/config/img_yolo_params.yaml")

    container = ComposableNodeContainer(
        name=[cam_ns, "_pipe"],
        namespace=cam_ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hdr_camera",
                plugin="CamStreamer",
                name=[cam_ns, "_driver_node"],
                parameters=[cam_far_params, {"camera_ns": cam_ns}],
                remappings=[],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name=[cam_ns, "_ori_resize_node"],
                parameters=[img_resize_params, img_ffmpeg_params],
                remappings=[
                    ("/image/image_raw", ["/", cam_ns, "/image"]),
                    ("/image/camera_info", ["/", cam_ns, "/camera_info"]),
                    ("/resize/image_raw", ["/", cam_ns, "/resized/image"]),
                    ("/resize/camera_info", ["/", cam_ns, "/resized/camera_info"]),
                    ("/resize/image_raw/compressed", ["/", cam_ns, "/resized/image/compressed"]),
                    ("/resize/image_raw/compressedDepth", ["/", cam_ns, "/resized/image/compressedDepth"]),
                    ("/resize/image_raw/theora", ["/", cam_ns, "/resized/image/theora"]),
                    ("/resize/image_raw/ffmpeg", ["/", cam_ns, "/resized/image/ffmpeg"]),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="topic_tools",
                plugin="topic_tools::ThrottleNode",
                name=[cam_ns, "_ori_img_throttle_node"],
                parameters=[
                    {"input_topic": ["/", cam_ns, "/image"]},
                    {"output_topic": ["/", cam_ns, "/throttled/image"]},
                    img_throttle_params,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name=[cam_ns, "_ori_thr_resize_node"],
                parameters=[img_resize_params, img_ffmpeg_params],
                remappings=[
                    ("/image/image_raw", ["/", cam_ns, "/throttled/image"]),
                    ("/image/camera_info", ["/", cam_ns, "/camera_info"]),
                    ("/resize/image_raw", ["/", cam_ns, "/throttled/resized/image"]),
                    ("/resize/camera_info", ["/", cam_ns, "/throttled/resized/camera_info"]),
                    ("/resize/image_raw/compressed", ["/", cam_ns, "/throttled/resized/image/compressed"]),
                    ("/resize/image_raw/compressedDepth", ["/", cam_ns, "/throttled/resized/image/compressedDepth"]),
                    ("/resize/image_raw/theora", ["/", cam_ns, "/throttled/resized/image/theora"]),
                    ("/resize/image_raw/ffmpeg", ["/", cam_ns, "/throttled/resized/image/ffmpeg"]),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="yolov8",
                plugin="Yolov8Detector",
                name=[cam_ns, "_yolov8_detector_node"],
                parameters=[img_yolo_params],
                remappings=[
                    ("/yolov8_detector/image", ["/", cam_ns, "/resized/image"]),
                    ("/yolov8_detector/image_with_detection", ["/", cam_ns, "/resized/image/detection"]),
                    ("/yolov8_detector/detection", ["/", cam_ns, "/resized/detection"]),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([container])
