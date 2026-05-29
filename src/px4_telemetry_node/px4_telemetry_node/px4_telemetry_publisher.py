import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition


class PX4TelemetryPublisher(Node):

    def __init__(self):
        super().__init__('px4_telemetry_publisher')

        self.telemetry_pub = self.create_publisher(
            String,
            'telemetry_data',
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            qos_profile
        )

        self.get_logger().info("PX4 real telemetry publisher started")

    def local_position_callback(self, msg):
        altitude = round(-msg.z, 2)

        velocity = math.sqrt(
            msg.vx ** 2 +
            msg.vy ** 2 +
            msg.vz ** 2
        )

        telemetry = {
            "altitude": altitude,
            "velocity": round(velocity, 2),
            "battery": "--",
            "flight_mode": "PX4_REAL"
        }

        out_msg = String()
        out_msg.data = json.dumps(telemetry)

        self.telemetry_pub.publish(out_msg)

        self.get_logger().info(f"PX4 telemetry: {out_msg.data}")


def main(args=None):
    rclpy.init(args=args)

    node = PX4TelemetryPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()