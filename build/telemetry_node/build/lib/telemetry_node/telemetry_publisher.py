import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random


class TelemetryPublisher(Node):

    def __init__(self):

        super().__init__('telemetry_publisher')

        self.publisher_ = self.create_publisher(
            String,
            'telemetry_data',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_telemetry
        )

    def publish_telemetry(self):

        telemetry = {
            "altitude": round(random.uniform(95, 105), 2),
            "velocity": round(random.uniform(10, 15), 2),
            "battery": round(random.uniform(80, 100), 2),
            "flight_mode": "OFFBOARD"
        }

        msg = String()

        msg.data = json.dumps(telemetry)

        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing telemetry")


def main(args=None):

    rclpy.init(args=args)

    node = TelemetryPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()