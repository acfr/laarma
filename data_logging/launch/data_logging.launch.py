from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="data_logging",
                executable="basic_recorder",
                name="basic_recorder",
                parameters=[
                    {
                        "logging_dir": "/mnt/ssd/logging",
                        "max_bag_size": 0,
                        "max_bag_duration": 60,
                        "name": "laarma_throttled",
                        "storage_budget": 800,
                        "sync": True,
                        # "config_file": "/mnt/ssd/config/basic.yaml"
                    }
                ],
            ),
            Node(
                package="data_logging",
                executable="event_recorder",
                name="event_recorder",
                parameters=[
                    {
                        "logging_dir": "/mnt/ssd/logging",
                        "max_cache_size": 200000000,
                        "name": "laarma_event",
                        "snapshot_delay": 10,
                        "snapshot_trigger_topic": "/event_trigger/event",
                        "storage_budget": 800,
                        "sync": True,
                        # "config_file": "/mnt/ssd/config/event.yaml"
                    }
                ],
            ),
        ]
    )
