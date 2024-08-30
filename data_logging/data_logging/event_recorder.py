import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from data_logging.base import RecorderNode
from rosbag2_py._storage import ConverterOptions, StorageOptions
from std_msgs.msg import Bool


class EventRecorder(RecorderNode):
    def __init__(self):
        super().__init__("recording_event")

        # Config setup
        default_config = Path(get_package_share_directory("data_logging"), "config", "event.yaml")
        self.declare_parameter("config_file", str(default_config))
        config_path = self.get_parameter("config_file").value

        # Snapshot timing management
        self.snapshot_available = True
        self.last_snapshot = self.get_clock().now()
        self.event_timer = None
        self.event_sub = self.create_subscription(Bool, self.snapshot_topic, self.eventCb, 1)

        self.loadConfig(config_path)
        self.start()

    def start(self):
        output = self.getTimestampedFileName()
        storage = StorageOptions(
            uri=str(output),
            snapshot_mode=True,
            max_cache_size=self.max_cache_size,
            storage_preset_profile="zstd_fast",
        )
        converter = ConverterOptions("", "")
        self.createWriter(storage, converter)
        self.get_logger().info(f"Recording to {str(output)}")

    def eventCb(self, msg):
        """Delay snapshot by half the specified bag duration to capture future data."""
        if msg.data and self.snapshot_available:
            self.get_logger().info(f"Event monitor triggered")
            self.snapshot_available = False
            self.event_timer = self.create_timer(self.snapshot_delay, self.snapshot)

    def snapshot(self):
        # Destroy the timer so the fn is not called again
        if self.event_timer:
            self.event_timer.destroy()

        # Dump the circular buffer to current bag, and start a new bag
        if self.writer:
            self.writer.take_snapshot()
            self.writer.split_bagfile()

        self.snapshot_available = True
        self.last_snapshot = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    event = EventRecorder()

    try:
        rclpy.spin(event)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        event.shutdown()
        event.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
