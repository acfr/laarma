import sys
from datetime import datetime
from pathlib import Path

import rclpy
import rosbag2_py as bag
from ament_index_python.packages import get_package_share_directory

from data_logging.base import RecorderNode
from data_logging.video_recorder import MP4Recorder


class BasicRecorder(RecorderNode):
    def __init__(self):
        super().__init__("recording_basic")

        self.restart_timer = self.create_timer(60, self.restartCb)

        # Config setup
        default_config = Path(get_package_share_directory("data_logging"), "config", "basic.yaml")
        self.declare_parameter("config_file", str(default_config))
        config_path = self.get_parameter("config_file").value

        self.loadConfig(config_path)
        self.start()

    def start(self):
        output = self.getTimestampedFileName()
        storage = bag._storage.StorageOptions(
            uri=str(output),
            max_bagfile_size=self.max_bag_size,
            max_bagfile_duration=self.max_bag_duration,
            storage_preset_profile="zstd_fast",
        )
        converter = bag._storage.ConverterOptions("", "")
        self.createWriter(storage, converter)
        self.createVideo(str(output))
        self.topicMonitorCb()
        self.get_logger().info(f"Recording to {str(output)}")

    def createVideo(self, filename: str):
        if video_topic := self.config.get("video"):
            topic_list = dict(self.get_topic_names_and_types())
            if video_topic in topic_list:
                video_output = Path(self.logging_dir, self.name, filename + ".mp4")
                self.video = MP4Recorder(video_topic, video_output)
            else:
                self.get_logger().warning(f"Topic {video_topic} does not exist, failed to stream video.")

    def restartCb(self):
        """Restart recording sessions every hour to make it easier to search for bags."""
        if datetime.now().minute == 0:
            self.shutdown()
            self.start()


def main(args=None):
    rclpy.init(args=args)
    basic = BasicRecorder()

    try:
        rclpy.spin(basic)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        basic.shutdown()
        basic.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
