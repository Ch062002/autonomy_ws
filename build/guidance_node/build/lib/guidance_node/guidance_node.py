import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GuidanceNode(Node):

    def __init__(self):
        super().__init__("guidance_node")

        self.mission_sub = self.create_subscription(
            String,
            "mission_upload",
            self.mission_callback,
            10
        )

        self.guidance_pub = self.create_publisher(
            String,
            "guidance_output",
            10
        )

        self.timer = self.create_timer(1.0, self.publish_guidance_output)

        self.mission = None
        self.guidance_mode = "LOS_GUIDANCE"
        self.active_segment = 0

        self.get_logger().info("Guidance Node started")
        self.get_logger().info("Mode: LOS_GUIDANCE")

    def mission_callback(self, msg):
        try:
            self.mission = json.loads(msg.data)
            self.active_segment = 0

            waypoints = self.mission.get("waypoints", [])

            self.get_logger().info("Mission received by Guidance Node")
            self.get_logger().info(f"Waypoint count: {len(waypoints)}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse mission: {e}")

    def compute_distance(self, p1, p2):
        dx = p2["lat"] - p1["lat"]
        dy = p2["lon"] - p1["lon"]
        dz = p2["alt"] - p1["alt"]

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def compute_los_guidance(self):
        if self.mission is None:
            return {
                "guidance_mode": self.guidance_mode,
                "status": "waiting_for_mission",
                "active_segment": None,
                "target_waypoint": None,
                "cross_track_error": None,
                "lookahead_point": None
            }

        waypoints = self.mission.get("waypoints", [])

        if len(waypoints) < 2:
            return {
                "guidance_mode": self.guidance_mode,
                "status": "need_at_least_two_waypoints",
                "active_segment": None,
                "target_waypoint": None,
                "cross_track_error": None,
                "lookahead_point": None
            }

        start_wp = waypoints[self.active_segment]
        end_wp = waypoints[min(self.active_segment + 1, len(waypoints) - 1)]

        path_length = self.compute_distance(start_wp, end_wp)

        lookahead_ratio = 0.4

        lookahead_point = {
            "lat": start_wp["lat"] + lookahead_ratio * (end_wp["lat"] - start_wp["lat"]),
            "lon": start_wp["lon"] + lookahead_ratio * (end_wp["lon"] - start_wp["lon"]),
            "alt": start_wp["alt"] + lookahead_ratio * (end_wp["alt"] - start_wp["alt"])
        }

        cross_track_error = 0.0

        return {
            "guidance_mode": self.guidance_mode,
            "status": "active",
            "active_segment": self.active_segment + 1,
            "target_waypoint": end_wp,
            "lookahead_point": lookahead_point,
            "path_length": round(path_length, 6),
            "cross_track_error": round(cross_track_error, 3)
        }

    def publish_guidance_output(self):
        guidance = self.compute_los_guidance()

        msg = String()
        msg.data = json.dumps(guidance)

        self.guidance_pub.publish(msg)

        self.get_logger().info(f"Guidance Output: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    node = GuidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()