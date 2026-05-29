import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleCommand


class PX4CommandNode(Node):

    def __init__(self):
        super().__init__("px4_command_node")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.command_publisher = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )

        self.get_logger().info("PX4 Command Node started")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.command_publisher.publish(msg)

        self.get_logger().info(
            f"Sent command: {command}, param1={param1}, param2={param2}"
        )

    def arm(self):
        for _ in range(10):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0
            )
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

    def disarm(self):
        for _ in range(10):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0
            )
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    node = PX4CommandNode()

    time.sleep(1.0)

    command_arg = sys.argv[1] if len(sys.argv) > 1 else "arm"

    if command_arg == "arm":
        node.get_logger().info("Sending repeated ARM commands...")
        node.arm()

    elif command_arg == "disarm":
        node.get_logger().info("Sending repeated DISARM commands...")
        node.disarm()

    else:
        node.get_logger().error("Invalid command. Use: arm or disarm")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()