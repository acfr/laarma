import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import requests
from std_msgs.msg import Bool


class Gpio(Node):

    def __init__(self):
        super().__init__("event_output_gpio")

        self.declare_parameter("use_system_default_qos", True)
        self.param_use_sys_default_qos = self.get_parameter("use_system_default_qos").get_parameter_value().bool_value

        selected_qos = QoSPresetProfiles.SENSOR_DATA.value
        if self.param_use_sys_default_qos:
            selected_qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.event_signal_subscription = self.create_subscription(
            Bool, f"/event_trigger/event_signal", self.msg_callback, qos_profile=selected_qos
        )
        self.last_msg_stamp = self.get_clock().now()  # initialise

        self.declare_parameter("router_ip", "https://192.168.1.1")
        self.declare_parameter("router_user", "user")
        self.declare_parameter("router_pwd", "1Password")
        self.declare_parameter("router_pin", "dout1")
        self.param_router_ip = self.get_parameter("router_ip").get_parameter_value().string_value
        self.param_router_user = self.get_parameter("router_user").get_parameter_value().string_value
        self.param_router_pwd = self.get_parameter("router_pwd").get_parameter_value().string_value
        self.param_router_pin = self.get_parameter("router_pin").get_parameter_value().string_value
        self.event_signal_pre = "undefined"
        requests.urllib3.disable_warnings(
            requests.packages.urllib3.exceptions.InsecureRequestWarning
        )  # Disable SSL warning

        self.get_logger().info("Start processing")

    def msg_callback(self, msg):
        self.get_logger().debug(f"I received: {msg.data}")
        self.last_msg_stamp = self.get_clock().now()

        if msg.data:
            self.set_router_pin("off")  # off means HIGH output
        else:
            self.set_router_pin("on")  # on means LOW output

    def set_router_pin(self, state: str):
        if self.event_signal_pre == state:
            return

        url = f"{self.param_router_ip}/cgi-bin/io_state?username={self.param_router_user}&password={self.param_router_pwd}&pin={self.param_router_pin}&state={state}"
        self.get_logger().debug(f"Sending: {url}")

        try:
            requests.get(url, verify=False)
            self.event_signal_pre = state
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Router GPIO operation failed")

    def __del__(self):
        self.set_router_pin("on")  # on means LOW output
        self.get_logger().info(f"Node cleans up")


def main(args=None):
    rclpy.init(args=args)

    gpio = Gpio()

    rclpy.spin(gpio)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gpio.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
