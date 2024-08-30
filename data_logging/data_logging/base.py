import importlib
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rosbag2_py as bag
import yaml
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.subscription import Subscription
from rosbag2_py._storage import ConverterOptions, StorageOptions


@dataclass
class Topic:
    name: str
    dtype: Optional[str] = None
    schedule: List[str] = field(default_factory=list)
    sub: Optional[Subscription] = None
    recording: bool = False


class RecorderNode(Node):
    def __init__(self, name: str):
        super().__init__(f"recorder_{name}")

        # Set params
        self.declare_parameter("logging_dir", "/mnt/ssd/logging")
        self.declare_parameter("max_bag_duration", 0)
        self.declare_parameter("max_bag_size", 0)
        self.declare_parameter("max_cache_size", 100000000)
        self.declare_parameter("name", "its")
        self.declare_parameter("snapshot_delay", 10)
        self.declare_parameter("snapshot_trigger_topic", "/event")
        self.declare_parameter("storage_budget", 1000)
        self.declare_parameter("sync", True)

        # Get params
        self.logging_dir = self.get_parameter("logging_dir").value
        self.max_bag_duration = self.get_parameter("max_bag_duration").value
        self.max_bag_size = self.get_parameter("max_bag_size").value
        self.max_cache_size = self.get_parameter("max_cache_size").value
        self.name = self.get_parameter("name").value
        self.snapshot_delay = self.get_parameter("snapshot_delay").value
        self.snapshot_topic = self.get_parameter("snapshot_trigger_topic").value
        self.storage_budget = self.get_parameter("storage_budget").value
        self.sync = self.get_parameter("sync").value

        # Setup paths
        if not Path(self.logging_dir).exists():
            self.get_logger().fatal("Logging directory does not exist")
            raise SystemExit
        self.stream_dir = Path(self.logging_dir, self.name)
        self.stream_dir.mkdir(exist_ok=True)

        # Class variables
        self.topics = []
        self.video = None
        self.writer = None
        self.qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # Optional time alignment
        self.get_logger().info("Aligning node to nearest minute based on system clock")
        if self.sync:
            while datetime.now().second != 0:
                pass
        self.topic_monitor_timer = self.create_timer(60, self.topicMonitorCb)
        self.disk_monitor_timer = self.create_timer(3600, self.diskMonitorCb)
        self.diskMonitorCb()

    def createWriter(self, storage: StorageOptions, converter: ConverterOptions):
        """Create a sequential writer."""
        self.writer = bag.SequentialWriter()
        self.writer.open(storage, converter)

    def shutdown(self):
        """Close the writer and save associated metadata."""
        if self.writer:
            self.writer.close()
            # Required when restarting the writer
            for topic in self.topics:
                if topic.sub is not None:
                    self.destroy_subscription(topic.sub)
                    topic.sub = None
                    topic.recording = False
        if self.video:
            self.video.terminate()

    def loadConfig(self, config_path: str):
        with open(config_path, "r") as file:
            try:
                self.get_logger().info(f"Loading config from file: {config_path}")
                self.config = yaml.safe_load(file)
            except yaml.YAMLError as e:
                self.get_logger().fatal("YAML error")
                print(e)
                raise SystemExit

        # Parse list of topics if available
        if topics := self.config.get("topics"):
            for topic in topics:
                if isinstance(topic, str):
                    topic = [topic]
                name = topic[0]
                dtype = self.getTopicDtype(name)
                if len(topic) > 1:
                    t = Topic(name, dtype, schedule=topic[1:])
                else:
                    t = Topic(name, dtype)
                self.topics.append(t)

    def makeCb(self, topic):
        def callback(msg):
            self.writer.write(topic, serialize_message(msg), self.get_clock().now().nanoseconds)

        return callback

    def scheduled(self, topic: Topic) -> bool:
        """
        Schedules formatted as: [11-14, 16-23, ...],
        return a bool that is True if the system time falls within the schedule.
        Do not wrap time around to the next day: [19-3]
        Examples:
        [0-1] = Records from midnight to 1AM
        [22-24] = Records from 10PM to midnight
        [9-24, 0-2] = Records from 9AM to 2AM
        """
        schedules = topic.schedule
        confirm = False
        hour = datetime.now().hour

        if not schedules:
            # If no schedule exists
            confirm = True
        else:
            for schedule in schedules:
                t0, t1 = schedule.split("-")
                t0, t1 = int(t0), int(t1)

                # Edge case if time wrap is included
                if t0 > t1:
                    self.get_logger().warning(f"Skipping scheduled time because it is not ascending: {schedule}")
                    continue

                # Edge case if recording continues to midnight
                if t1 == 0:
                    t1 = 24

                if t0 <= hour < t1:
                    confirm = True
        return confirm

    def topicMonitorCb(self):
        """Add or remove topics from the writer based on a schedule."""
        for topic in self.topics:
            # Check if a new topic is being published
            if topic.dtype is None:
                topic.dtype = self.getTopicDtype(topic.name)
            if topic.dtype:
                if self.scheduled(topic) and not topic.recording:
                    topic = self.addTopic(topic)
                elif not self.scheduled(topic) and topic.recording:
                    topic = self.removeTopic(topic)

    def addTopic(self, topic: Topic) -> Topic:
        """Add topic to the writer."""
        topic_class = self.getTopicModule(topic)
        metadata = bag._storage.TopicMetadata(name=topic.name, type=topic.dtype, serialization_format="cdr")
        self.writer.create_topic(metadata)
        topic.sub = self.create_subscription(topic_class, topic.name, self.makeCb(topic.name), qos_profile=self.qos)
        topic.recording = True
        return topic

    def removeTopic(self, topic: Topic) -> Topic:
        """Remove topic from the writer."""
        metadata = bag._storage.TopicMetadata(name=topic.name, type=topic.dtype, serialization_format="cdr")
        self.writer.remove_topic(metadata)
        self.destroy_subscription(topic.sub)
        topic.sub = None
        topic.recording = False
        return topic

    def getTopicDtype(self, topic_name: str) -> Optional[str]:
        """Returns topic dtype. i.e. sensor_msgs/msg/Image"""
        topic_list = dict(self.get_topic_names_and_types())
        if topic_name not in topic_list:
            self.get_logger().warning(f"Topic {topic_name} does not exist, skipping")
            return None
        return topic_list[topic_name][0]

    def getTopicModule(self, topic: Topic):
        """Dynamically import the module required to load the topic."""
        # Seperate topic import into module and type (sensor_msgs/msg/Image -> sensor_msgs.msg and Image)
        topic_module = ".".join(topic.dtype.split("/")[:-1])
        topic_type = topic.dtype.split("/")[-1]

        # Dynamic msg import (from topic_module import topic_type)
        try:
            topic_class = getattr(importlib.import_module(topic_module), topic_type)
            return topic_class
        except:
            self.get_logger().fatal(f"Cannot import {topic_module} module")
            raise SystemExit

    def getDiskUsage(self):
        """Get disk usage percentage."""
        # NOTE shutil.disk_usage does not provide an accurate `used` value
        total, used, free = shutil.disk_usage(self.stream_dir)
        return (total - free) / total

    def getStreamDiskUsage(self):
        """Get disk usage for current stream, in bytes."""
        return sum(f.stat().st_size for f in self.stream_dir.glob("**/*") if f.is_file())

    def diskMonitorCb(self):
        """Prevent disk from running out of space by deleting old data and limiting storage allocation"""
        while (self.getDiskUsage() >= 0.95) or ((self.getStreamDiskUsage() / 1e9) >= self.storage_budget):
            self.get_logger().warning("Disk usage exceeded 95% or storage budget.")
            self.deleteOldestData()

        if self.getDiskUsage() >= 0.85:
            self.get_logger().warning("Disk usage exceeded 85%.")

    def deleteOldestData(self):
        data = [x for x in self.stream_dir.glob("*") if not x.name.endswith(".mp4")]
        data.sort()
        oldest_file = data[0]
        shutil.rmtree(oldest_file)
        self.get_logger().warning(f"Deleted {oldest_file}")

    def getTimestampedFileName(self) -> Path:
        """Generate a timestamped file name (logging_dir/name/YYMMDD_HHMMSS_name)"""
        timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
        bagname = f"{timestamp}_{self.name}"
        return Path(self.logging_dir, self.name, bagname)
