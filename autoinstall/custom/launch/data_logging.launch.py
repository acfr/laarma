from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="data_logging",
                executable="basic_recorder",
                name="basic_recorder",
                parameters=[
                    {
                        "name": "laarma_throttled",
                        "logging_dir": "/mnt/ssd/logging",
                        "storage_budget": 1400,
                        "sync": True,
                        # Node specific params
                        "config_file": "/mnt/config/logging_throttled_topics.yaml",
                        "max_bag_duration": 60,
                        "max_bag_size": 0,
                    }
                ],
            ),
            Node(
                package="data_logging",
                executable="basic_recorder",
                name="basic_recorder_telemetry",
                parameters=[
                    {
                        "name": "laarma_telemetry",
                        "logging_dir": "/mnt/ssd/logging",
                        "storage_budget": 100,
                        "sync": True,
                        # Node specific params
                        "config_file": "/mnt/config/logging_telemetry_topics.yaml",
                        "max_bag_duration": 0,
                        "max_bag_size": 0,
                    }
                ],
            ),
            Node(
                package="data_logging",
                executable="event_recorder",
                name="event_recorder",
                parameters=[
                    {
                        "name": "laarma_event",
                        "logging_dir": "/mnt/ssd/logging",
                        "storage_budget": 300,
                        "sync": True,
                        # Node specific params
                        "config_file": "/mnt/config/logging_event_topics.yaml",
                        "max_cache_size": 120000000,
                        "snapshot_delay": 10,
                        "snapshot_trigger_topic": "/event_trigger/event",
                    }
                ],
            ),
        ]
    )
