#!/usr/bin/env python3

import rosbag2_py  # use the rosbag2_py library along with the MCAP Storage Plugin to interact with MCAP files in ROS2 packages: sudo apt install ros-iron-ros2bag && sudo apt install ros-iron-rosbag2-transport && sudo apt install ros-iron-rosbag2-storage-mcap
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from datetime import datetime


def bag_tagger(input_bag_path, params):

    # print(f'start reading {input_bag_path}')

    # create reader instance and open for reading
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
    except:
        del reader
        return False

    topic_types = reader.get_all_topics_and_types()

    nDet = {}
    for topic_type in topic_types:
        if topic_type.type == "vision_msgs/msg/Detection2DArray":
            nDet[topic_type.name] = 0

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        msg_typename = typename(topic)
        msg_type = get_message(msg_typename)
        msg = deserialize_message(data, msg_type)

        if msg_typename == "vision_msgs/msg/Detection2DArray":
            nObjects = len(msg.detections)
            # print(f'objects in received detection msg: {nObjects}')

            threshold = params["rgb_cam_threshold"]
            if topic.find("thermal") != -1:
                threshold = params["thermal_threshold"]

            for detection in msg.detections:
                for result in detection.results:
                    for coi in params["classes_of_interest"]:
                        if result.hypothesis.class_id == coi:
                            # print(f'score: {result.hypothesis.score}')
                            if result.hypothesis.score >= threshold:
                                nDet[topic] += 1

    del reader

    # print(nDet)

    ret = False
    for key in nDet:
        if nDet[key] > params["count_threshold"]:
            ret = True
            break
    return ret
