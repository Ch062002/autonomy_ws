import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PX4MissionNode(Node):

    def __init__(self):
        super().__init__("px4_mission_node")

        self.mission_subscriber = self.create_subscription(
            String,
            "mission_upload",
            self.mission_callback,
            10
        )

        self.get_logger().info("PX4 Mission Node started")
        self.get_logger().info("Waiting for mission_upload messages...")

    def mission_callback(self, msg):
        try:
            mission = json.loads(msg.data)

            self.get_logger().info("Mission received successfully")
            self.get_logger().info(f"Mission Name: {mission.get('name')}")
            self.get_logger().info(f"Waypoint Count: {len(mission.get('waypoints', []))}")

            for index, wp in enumerate(mission.get("waypoints", []), start=1):
                self.get_logger().info(
                    f"WP{index}: lat={wp.get('lat')}, lon={wp.get('lon')}, alt={wp.get('alt')}"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to parse mission: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = PX4MissionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()