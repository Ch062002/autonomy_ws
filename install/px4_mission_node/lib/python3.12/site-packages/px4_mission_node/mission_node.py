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

    def convert_to_px4_mission_items(self, mission):
        px4_items = []

        waypoints = mission.get("waypoints", [])

        for index, wp in enumerate(waypoints):
            item = {
                "seq": index,
                "command": "NAV_WAYPOINT",
                "frame": "GLOBAL_RELATIVE_ALT",
                "latitude": float(wp.get("lat")),
                "longitude": float(wp.get("lon")),
                "altitude": float(wp.get("alt")),
                "acceptance_radius": 5.0,
                "hold_time": 0.0
            }

            px4_items.append(item)

        return px4_items

    def mission_callback(self, msg):
        try:
            mission = json.loads(msg.data)

            mission_name = mission.get("name", "Unnamed Mission")
            waypoints = mission.get("waypoints", [])

            if len(waypoints) == 0:
                self.get_logger().warn("Mission rejected: no waypoints found")
                return

            px4_items = self.convert_to_px4_mission_items(mission)

            self.get_logger().info("Mission received successfully")
            self.get_logger().info(f"Mission Name: {mission_name}")
            self.get_logger().info(f"Waypoint Count: {len(waypoints)}")

            self.get_logger().info("Converted PX4 Mission Items:")

            for item in px4_items:
                self.get_logger().info(
                    f"SEQ {item['seq']} | CMD {item['command']} | "
                    f"LAT {item['latitude']} | LON {item['longitude']} | "
                    f"ALT {item['altitude']} m"
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