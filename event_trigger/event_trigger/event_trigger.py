from datetime import datetime
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import requests
from std_msgs.msg import Bool, Float32
import yaml

import Jetson.GPIO as GPIO  # use Teltonika GPIO


class EventTrigger(Node):

    def __init__(self):
        super().__init__("event_trigger")

        self.declare_parameter("use_system_default_qos", True)
        self.declare_parameter("event_signal_duration", 30)
        self.declare_parameter("event_signal_output_pin", 13)
        self.param_use_sys_default_qos = self.get_parameter("use_system_default_qos").get_parameter_value().bool_value
        self.param_event_signal_duration = (
            self.get_parameter("event_signal_duration").get_parameter_value().integer_value
        )
        self.param_event_signal_output_pin = (
            self.get_parameter("event_signal_output_pin").get_parameter_value().integer_value
        )
        default_signal_log = Path("/mnt/ssd/logging", "signal", "log.txt")
        self.declare_parameter("signal_log_file", str(default_signal_log))

        default_event_config = Path(
            get_package_share_directory("event_trigger"), "config", "event_trigger_params.yaml"
        )
        self.declare_parameter("event_config_file", str(default_event_config))
        event_config = self.get_parameter("event_config_file").get_parameter_value().string_value
        self.get_logger().info(f"Loading config from file: {event_config}")
        self.loadEventConfig(event_config)

        self.detection_triggers = self.config.get("detection_triggers")
        self.get_logger().info(f"detection_triggers: {self.detection_triggers}")
        nTriggers = len(self.detection_triggers)
        self.get_logger().info(f"Num of triggers: {nTriggers}")

        selected_qos = QoSPresetProfiles.SENSOR_DATA.value
        if self.param_use_sys_default_qos:
            selected_qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.estimate = {}  # Bayesian estimation results for each trigger
        self.last_msg_stamp = {}  # last msg timestamp for each trigger for timeout detection
        self.trigger_subscription = {}
        self.filtered_trigger_publisher = {}
        for trigger in self.detection_triggers:
            topic = trigger["topic"]
            self.get_logger().info(f"Subscribe to trigger: {topic}")
            self.trigger_subscription[topic] = self.create_subscription(
                Bool, topic, self.makeCallback(trigger), qos_profile=selected_qos
            )
            self.filtered_trigger_publisher[topic] = self.create_publisher(
                Float32, f"{topic}/filtered", qos_profile=selected_qos
            )
            self.estimate[topic] = np.array([[0.5], [0.5]])  # initialise the estimation
            self.last_msg_stamp[topic] = self.get_clock().now()  # initialise

        self.event_check_timer = self.create_timer(1, self.eventCheckTimerCb)
        self.event_publisher = self.create_publisher(Bool, "out/event", qos_profile=selected_qos)
        self.signal_countdown = 0
        self.event_signal_publisher = self.create_publisher(Bool, "out/event_signal", qos_profile=selected_qos)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.param_event_signal_output_pin, GPIO.OUT, initial=GPIO.LOW)

        self.declare_parameter("event_signal_output_router_ip", "https://192.168.1.1")
        self.declare_parameter("event_signal_output_router_user", "user")
        self.declare_parameter("event_signal_output_router_pwd", "1Password")
        self.declare_parameter("event_signal_output_router_pin", "dout1")
        self.param_router_ip = self.get_parameter("event_signal_output_router_ip").get_parameter_value().string_value
        self.param_router_user = (
            self.get_parameter("event_signal_output_router_user").get_parameter_value().string_value
        )
        self.param_router_pwd = self.get_parameter("event_signal_output_router_pwd").get_parameter_value().string_value
        self.param_router_pin = self.get_parameter("event_signal_output_router_pin").get_parameter_value().string_value
        self.event_signal_output_pre = "undefined"
        requests.urllib3.disable_warnings(
            requests.packages.urllib3.exceptions.InsecureRequestWarning
        )  # Disable SSL warning
        self.set_router_pin("on")  # on means LOW output, default

        self.get_logger().info("Start processing")

    def loadEventConfig(self, record_config):
        with open(record_config, "r") as file:
            try:
                self.config = yaml.safe_load(file)
            except yaml.YAMLError as e:
                self.get_logger().fatal("YAML error")
                print(e)
                raise SystemExit

    def makeCallback(self, trigger):
        def callback(msg):
            topic = trigger["topic"]
            self.get_logger().debug(f"I received: {msg.data} from {topic}")
            self.last_msg_stamp[topic] = self.get_clock().now()
            likelihood = np.array([[0.5], [0.5]])  # likelihood for no observation
            hour = datetime.now().hour
            self.get_logger().debug(f"Hour of the day: {hour}")
            for schedule in trigger["schedule"]:
                if hour >= schedule["from"] and hour < schedule["to"]:
                    index = 0 if msg.data else 1
                    likelihood = np.array(schedule["likelihood"])[
                        :, [index]
                    ]  # first or second column in the likelihood matrix
            self.get_logger().debug(f"likelihood: {likelihood} for {topic}")
            prediction = np.array(trigger["transition"]).dot(self.estimate[topic])
            posterior = likelihood * prediction
            posterior = posterior / sum(posterior)
            self.estimate[topic] = posterior
            self.get_logger().debug(f"posterior: {posterior} for {topic}")
            filtered = Float32()
            filtered.data = float(self.estimate[topic][0])
            self.filtered_trigger_publisher[topic].publish(filtered)

        return callback

    def eventCheckTimerCb(self):
        now = self.get_clock().now()
        if self.signal_countdown > 0:
            self.signal_countdown = self.signal_countdown - 1

        positive_event = Bool()
        positive_event.data = False
        for trigger in self.detection_triggers:
            topic = trigger["topic"]
            duration = now - self.last_msg_stamp[topic]
            if duration.nanoseconds / 1e9 > 1:  # trigger message timeout, something is wrong
                self.get_logger().debug(f"Messages from trigger {topic} time out")
                self.estimate[topic] = np.array([[0.5], [0.5]])  # reset estimate
            if self.estimate[topic][0] > trigger["threshold"]:
                positive_event.data = True
                self.signal_countdown = self.param_event_signal_duration
                break
        self.get_logger().debug(f"publish event: {positive_event.data}")
        self.event_publisher.publish(positive_event)

        event_signal = Bool()
        event_signal.data = False
        if self.signal_countdown > 0:
            event_signal.data = True
        self.get_logger().debug(f"publish event signal: {event_signal.data}, countdown: {self.signal_countdown}")
        self.event_signal_publisher.publish(event_signal)
        if event_signal.data:
            GPIO.output(self.param_event_signal_output_pin, GPIO.HIGH)
            self.set_router_pin("off")  # off means HIGH output, temp disabled
        else:
            GPIO.output(self.param_event_signal_output_pin, GPIO.LOW)
            self.set_router_pin("on")  # on means LOW output, temp disabled

    def set_router_pin(self, state: str):
        if self.event_signal_output_pre == state:
            return

        url = f"{self.param_router_ip}/cgi-bin/io_state?username={self.param_router_user}&password={self.param_router_pwd}&pin={self.param_router_pin}&state={state}"
        self.get_logger().debug(f"Sending: {url}")

        try:
            requests.get(url, verify=False)
            self.event_signal_output_pre = state
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Router GPIO operation failed")
            return

        ts = datetime.now().strftime("%y%m%d-%H%M%S")
        data = f"[{ts}] pin {state}"
        filename = self.get_parameter("signal_log_file").get_parameter_value().string_value
        if os.path.exists(filename):
            append_write = "a"  # append if already exists
        else:
            append_write = "w"  # make a new file if not
        with open(filename, append_write) as file:
            file.write(data + "\n")

    def __del__(self):
        GPIO.output(self.param_event_signal_output_pin, GPIO.LOW)
        GPIO.cleanup()
        self.set_router_pin("on")  # on means LOW output
        self.get_logger().info(f"Node cleans up")


def main(args=None):
    rclpy.init(args=args)

    event_trigger = EventTrigger()

    rclpy.spin(event_trigger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    event_trigger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
