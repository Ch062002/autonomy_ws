import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition


class GuidanceNode(Node):

    def __init__(self):
        super().__init__("guidance_node")

        self.mission_sub = self.create_subscription(
            String,
            "mission_upload",
            self.mission_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.global_position_callback,
            qos_profile
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

        self.current_position = {
            "lat": 47.3977,
            "lon": 8.5456,
            "alt": 0.0
        }

        self.get_logger().info("Guidance Node started")
        self.get_logger().info("Mode: LOS_GUIDANCE with cross-track error")

    def mission_callback(self, msg):
        try:
            self.mission = json.loads(msg.data)
            self.active_segment = 0

            waypoints = self.mission.get("waypoints", [])

            self.get_logger().info("Mission received by Guidance Node")
            self.get_logger().info(f"Waypoint count: {len(waypoints)}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse mission: {e}")

    def global_position_callback(self, msg):
        self.current_position = {
            "lat": float(msg.lat),
            "lon": float(msg.lon),
            "alt": float(msg.alt)
        }

    def latlon_to_local_meters(self, reference, point):
        earth_radius = 6378137.0

        ref_lat_rad = math.radians(reference["lat"])

        d_lat = math.radians(point["lat"] - reference["lat"])
        d_lon = math.radians(point["lon"] - reference["lon"])

        north = d_lat * earth_radius
        east = d_lon * earth_radius * math.cos(ref_lat_rad)
        down = reference["alt"] - point["alt"]

        return [north, east, down]

    def vector_norm(self, vector):
        return math.sqrt(sum(v * v for v in vector))

    def dot_product(self, a, b):
        return sum(x * y for x, y in zip(a, b))

    def compute_los_guidance(self):
        if self.mission is None:
            return {
                "guidance_mode": self.guidance_mode,
                "status": "waiting_for_mission",
                "active_segment": None,
                "cross_track_error": None,
                "along_track_distance": None,
                "lookahead_point": None,
                "current_position": self.current_position
            }

        waypoints = self.mission.get("waypoints", [])

        if len(waypoints) < 2:
            return {
                "guidance_mode": self.guidance_mode,
                "status": "need_at_least_two_waypoints",
                "active_segment": None,
                "cross_track_error": None,
                "along_track_distance": None,
                "lookahead_point": None,
                "current_position": self.current_position
            }

        start_wp = waypoints[self.active_segment]
        end_wp = waypoints[min(self.active_segment + 1, len(waypoints) - 1)]

        start_local = [0.0, 0.0, 0.0]
        end_local = self.latlon_to_local_meters(start_wp, end_wp)
        current_local = self.latlon_to_local_meters(start_wp, self.current_position)

        path_vector = [
            end_local[0] - start_local[0],
            end_local[1] - start_local[1],
            end_local[2] - start_local[2]
        ]

        current_vector = [
            current_local[0] - start_local[0],
            current_local[1] - start_local[1],
            current_local[2] - start_local[2]
        ]

        path_length = self.vector_norm(path_vector)

        if path_length < 1e-6:
            along_track_distance = 0.0
            cross_track_error = 0.0
            projection_ratio = 0.0
        else:
            path_unit = [v / path_length for v in path_vector]
            along_track_distance = self.dot_product(current_vector, path_unit)

            projection = [
                along_track_distance * path_unit[0],
                along_track_distance * path_unit[1],
                along_track_distance * path_unit[2]
            ]

            error_vector = [
                current_vector[0] - projection[0],
                current_vector[1] - projection[1],
                current_vector[2] - projection[2]
            ]

            cross_track_error = self.vector_norm(error_vector)
            projection_ratio = max(0.0, min(1.0, along_track_distance / path_length))

        lookahead_ratio = min(1.0, projection_ratio + 0.25)

        lookahead_point = {
            "lat": start_wp["lat"] + lookahead_ratio * (end_wp["lat"] - start_wp["lat"]),
            "lon": start_wp["lon"] + lookahead_ratio * (end_wp["lon"] - start_wp["lon"]),
            "alt": start_wp["alt"] + lookahead_ratio * (end_wp["alt"] - start_wp["alt"])
        }

        return {
            "guidance_mode": self.guidance_mode,
            "status": "active",
            "active_segment": self.active_segment + 1,
            "target_waypoint": end_wp,
            "lookahead_point": lookahead_point,
            "path_length": round(path_length, 2),
            "along_track_distance": round(along_track_distance, 2),
            "cross_track_error": round(cross_track_error, 2),
            "projection_ratio": round(projection_ratio, 3),
            "current_position": self.current_position
        }

    def publish_guidance_output(self):
        guidance = self.compute_los_guidance()

        msg = String()
        msg.data = json.dumps(guidance)

        self.guidance_pub.publish(msg)
        with open("/tmp/guidance_output.json", "w") as f:
            json.dump(guidance, f)
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