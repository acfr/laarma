import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


class DetectionTrigger(Node):

    def __init__(self):
        super().__init__("detection_trigger")

        self.declare_parameter("use_system_default_qos", True)
        self.declare_parameter("classes_of_interest", ["cassowary"])
        self.declare_parameter("coi_score_threshold", 0.6)
        self.declare_parameter("enable_bg_class_filter", False) # feature implemented but not enabled in the laarma project
        self.declare_parameter("bg_class_overlapping_threshold", 0.9)
        self.declare_parameter("background_classes", ["car"])
        self.declare_parameter("enable_roi", False)
        self.declare_parameter("region_of_interest", [0, 0, 640, 480])  # x1, y1, x2, y2
        self.declare_parameter("non_roi_score_threshold", 0.98)
        self.param_use_sys_default_qos = self.get_parameter("use_system_default_qos").get_parameter_value().bool_value
        self.param_classes_of_interest = self.get_parameter("classes_of_interest").value
        self.param_coi_score_threshold = self.get_parameter("coi_score_threshold").get_parameter_value().double_value
        self.param_enable_bg_class_filter = (
            self.get_parameter("enable_bg_class_filter").get_parameter_value().bool_value
        )
        self.param_bg_class_overlapping_threshold = (
            self.get_parameter("bg_class_overlapping_threshold").get_parameter_value().double_value
        )
        self.param_background_classes = self.get_parameter("background_classes").value
        self.param_enable_roi = self.get_parameter("enable_roi").get_parameter_value().bool_value
        self.param_region_of_interest = self.get_parameter("region_of_interest").value
        self.param_non_roi_score_threshold = (
            self.get_parameter("non_roi_score_threshold").get_parameter_value().double_value
        )

        selected_qos = QoSPresetProfiles.SENSOR_DATA.value
        if self.param_use_sys_default_qos:
            selected_qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.detection_subscription = self.create_subscription(
            Detection2DArray, "in/detection", self.detection_topic_callback, qos_profile=selected_qos
        )
        self.trigger_publisher = self.create_publisher(Bool, "out/trigger", qos_profile=selected_qos)

    def check_match(self, msg):
        for detection in msg.detections:
            if self.param_enable_roi:
                center = detection.bbox.center.position
                if (
                    center.x > self.param_region_of_interest[0]
                    and center.x < self.param_region_of_interest[2]
                    and center.y > self.param_region_of_interest[1]
                    and center.y < self.param_region_of_interest[3]
                ):
                    score_threshold = self.param_coi_score_threshold
                else:
                    score_threshold = self.param_non_roi_score_threshold
            else:
                score_threshold = self.param_coi_score_threshold
            for result in detection.results:
                if result.hypothesis.score < score_threshold:
                    continue
                for coi in self.param_classes_of_interest:
                    if result.hypothesis.class_id != coi:
                        continue
                    if self.param_enable_bg_class_filter:
                        if not self.check_background_classes(detection.bbox, msg):
                            return True
                    else:
                        return True
        return False

    def check_background_classes(self, coi_bbox, msg):
        for detection in msg.detections:
            for result in detection.results:
                for bg_class in self.param_background_classes:
                    if result.hypothesis.class_id != bg_class:
                        continue
                    overlapping = self.cal_overlapping(coi_bbox, detection.bbox)
                    self.get_logger().debug(
                        f"coi class with background class {bg_class} has overlapping score {overlapping}"
                    )
                    if overlapping > self.param_bg_class_overlapping_threshold:
                        self.get_logger().debug(f"coi detection ignored due to high overlapping with background class")
                        return True
        return False

    # calculate overlapping ratio of intersection of coi bbox and bg bbox over the coi bbox area
    def cal_overlapping(self, bbox_coi, bbox_bg):
        boxA_x1 = bbox_coi.center.position.x - bbox_coi.size_x / 2
        boxA_x2 = bbox_coi.center.position.x + bbox_coi.size_x / 2
        boxA_y1 = bbox_coi.center.position.y - bbox_coi.size_y / 2
        boxA_y2 = bbox_coi.center.position.y + bbox_coi.size_y / 2
        boxB_x1 = bbox_bg.center.position.x - bbox_bg.size_x / 2
        boxB_x2 = bbox_bg.center.position.x + bbox_bg.size_x / 2
        boxB_y1 = bbox_bg.center.position.y - bbox_bg.size_y / 2
        boxB_y2 = bbox_bg.center.position.y + bbox_bg.size_y / 2
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA_x1, boxB_x1)
        yA = max(boxA_y1, boxB_y1)
        xB = min(boxA_x2, boxB_x2)
        yB = min(boxA_y2, boxB_y2)

        # compute the area of intersection rectangle
        interArea = max((xB - xA, 0)) * max((yB - yA), 0)
        if interArea == 0:
            return 0
        # compute the area of coi bbox
        boxAArea = abs((boxA_x2 - boxA_x1) * (boxA_y2 - boxA_y1))

        overlapping = interArea / float(boxAArea)

        # return the overalpping ratio
        return overlapping

    def detection_topic_callback(self, msg):
        nObjects = len(msg.detections)
        self.get_logger().debug(f"objects in received detection msg: {nObjects}")
        match = self.check_match(msg)
        self.get_logger().debug(f"class match result: {match}")
        trigger_msg = Bool()
        trigger_msg.data = match
        self.trigger_publisher.publish(trigger_msg)


def main(args=None):
    rclpy.init(args=args)

    detection_trigger = DetectionTrigger()

    rclpy.spin(detection_trigger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection_trigger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
