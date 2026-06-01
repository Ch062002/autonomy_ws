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

        self.mode_sub = self.create_subscription(
            String,
            "guidance_mode",
            self.guidance_mode_callback,
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
        self.active_waypoint = 0

        self.current_position = {
            "lat": 47.3977,
            "lon": 8.5456,
            "alt": 0.0
        }

        self.get_logger().info("Guidance Node started")
        self.get_logger().info("Default Mode: LOS_GUIDANCE")

    def mission_callback(self, msg):
        try:
            self.mission = json.loads(msg.data)
            self.active_segment = 0
            self.active_waypoint = 0

            waypoints = self.mission.get("waypoints", [])

            self.get_logger().info("Mission received by Guidance Node")
            self.get_logger().info(f"Waypoint count: {len(waypoints)}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse mission: {e}")

    def guidance_mode_callback(self, msg):
        mode = msg.data.strip().upper()

        allowed_modes = [
            "DIRECT_WAYPOINT",
            "LOS_GUIDANCE",
            "PURE_PURSUIT",
            "VECTOR_FIELD",
            "DUBINS"
        ]

        if mode in allowed_modes:
            self.guidance_mode = mode
            self.get_logger().info(
                f"Guidance mode changed to: {self.guidance_mode}"
            )
        else:
            self.get_logger().warn(
                f"Invalid guidance mode received: {mode}"
            )

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

    def compute_bearing_deg(self, current, target):
        lat1 = math.radians(current["lat"])
        lat2 = math.radians(target["lat"])
        d_lon = math.radians(target["lon"] - current["lon"])

        y = math.sin(d_lon) * math.cos(lat2)
        x = (
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
        )

        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360.0) % 360.0

    def compute_direct_waypoint_guidance(self):
        if self.mission is None:
            return {
                "guidance_mode": "DIRECT_WAYPOINT",
                "status": "waiting_for_mission",
                "target_waypoint": None,
                "distance_to_target": None,
                "bearing_to_target": None,
                "altitude_error": None,
                "cross_track_error": None,
                "along_track_distance": None,
                "path_length": None,
                "current_position": self.current_position
            }

        waypoints = self.mission.get("waypoints", [])

        if len(waypoints) == 0:
            return {
                "guidance_mode": "DIRECT_WAYPOINT",
                "status": "no_waypoints",
                "target_waypoint": None,
                "distance_to_target": None,
                "bearing_to_target": None,
                "altitude_error": None,
                "cross_track_error": None,
                "along_track_distance": None,
                "path_length": None,
                "current_position": self.current_position
            }

        target = waypoints[min(self.active_waypoint, len(waypoints) - 1)]

        local_vector = self.latlon_to_local_meters(
            self.current_position,
            target
        )

        horizontal_distance = math.sqrt(
            local_vector[0] ** 2 + local_vector[1] ** 2
        )

        distance_to_target = self.vector_norm(local_vector)
        bearing_to_target = self.compute_bearing_deg(
            self.current_position,
            target
        )
        altitude_error = target["alt"] - self.current_position["alt"]

        return {
            "guidance_mode": "DIRECT_WAYPOINT",
            "status": "active",
            "target_waypoint_index": self.active_waypoint + 1,
            "target_waypoint": target,
            "distance_to_target": round(distance_to_target, 2),
            "horizontal_distance": round(horizontal_distance, 2),
            "bearing_to_target": round(bearing_to_target, 2),
            "altitude_error": round(altitude_error, 2),
            "cross_track_error": None,
            "along_track_distance": None,
            "path_length": None,
            "current_position": self.current_position
        }

    def compute_los_guidance(self):
        if self.mission is None:
            return {
                "guidance_mode": "LOS_GUIDANCE",
                "status": "waiting_for_mission",
                "active_segment": None,
                "cross_track_error": None,
                "along_track_distance": None,
                "path_length": None,
                "lookahead_point": None,
                "current_position": self.current_position
            }

        waypoints = self.mission.get("waypoints", [])

        if len(waypoints) < 2:
            return {
                "guidance_mode": "LOS_GUIDANCE",
                "status": "need_at_least_two_waypoints",
                "active_segment": None,
                "cross_track_error": None,
                "along_track_distance": None,
                "path_length": None,
                "lookahead_point": None,
                "current_position": self.current_position
            }

        start_wp = waypoints[self.active_segment]
        end_wp = waypoints[min(self.active_segment + 1, len(waypoints) - 1)]

        start_local = [0.0, 0.0, 0.0]
        end_local = self.latlon_to_local_meters(start_wp, end_wp)
        current_local = self.latlon_to_local_meters(
            start_wp,
            self.current_position
        )

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
            along_track_distance = self.dot_product(
                current_vector,
                path_unit
            )

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
            projection_ratio = max(
                0.0,
                min(1.0, along_track_distance / path_length)
            )

        lookahead_ratio = min(1.0, projection_ratio + 0.25)

        lookahead_point = {
            "lat": start_wp["lat"]
            + lookahead_ratio * (end_wp["lat"] - start_wp["lat"]),
            "lon": start_wp["lon"]
            + lookahead_ratio * (end_wp["lon"] - start_wp["lon"]),
            "alt": start_wp["alt"]
            + lookahead_ratio * (end_wp["alt"] - start_wp["alt"])
        }

        return {
            "guidance_mode": "LOS_GUIDANCE",
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

    def compute_pure_pursuit_guidance(self):
        if self.mission is None:
            return {
                "guidance_mode": "PURE_PURSUIT",
                "status": "waiting_for_mission"
            }

        waypoints = self.mission.get("waypoints", [])

        if len(waypoints) < 2:
            return {
                "guidance_mode": "PURE_PURSUIT",
                "status": "need_at_least_two_waypoints"
            }

        start_wp = waypoints[self.active_segment]
        end_wp = waypoints[min(self.active_segment + 1, len(waypoints) - 1)]

        start_local = [0.0, 0.0, 0.0]
        end_local = self.latlon_to_local_meters(start_wp, end_wp)
        current_local = self.latlon_to_local_meters(
            start_wp,
            self.current_position
        )

        path_vector = [
            end_local[0] - start_local[0],
            end_local[1] - start_local[1],
            end_local[2] - start_local[2]
        ]

        path_length = self.vector_norm(path_vector)

        if path_length < 1e-6:
            return {
                "guidance_mode": "PURE_PURSUIT",
                "status": "invalid_path"
            }

        path_unit = [v / path_length for v in path_vector]

        current_vector = [
            current_local[0] - start_local[0],
            current_local[1] - start_local[1],
            current_local[2] - start_local[2]
        ]

        along_track_distance = self.dot_product(
            current_vector,
            path_unit
        )

        lookahead_distance = 20.0

        pursuit_distance = max(
            0.0,
            min(path_length, along_track_distance + lookahead_distance)
        )

        pursuit_ratio = pursuit_distance / path_length

        lookahead_point = {
            "lat": start_wp["lat"]
            + pursuit_ratio * (end_wp["lat"] - start_wp["lat"]),
            "lon": start_wp["lon"]
            + pursuit_ratio * (end_wp["lon"] - start_wp["lon"]),
            "alt": start_wp["alt"]
            + pursuit_ratio * (end_wp["alt"] - start_wp["alt"])
        }

        pursuit_vector = self.latlon_to_local_meters(
            self.current_position,
            lookahead_point
        )

        pursuit_distance_actual = self.vector_norm(pursuit_vector)

        pursuit_heading = math.degrees(
            math.atan2(pursuit_vector[1], pursuit_vector[0])
        )

        return {
            "guidance_mode": "PURE_PURSUIT",
            "status": "active",
            "active_segment": self.active_segment + 1,
            "lookahead_distance": lookahead_distance,
            "lookahead_point": lookahead_point,
            "pursuit_distance": round(pursuit_distance_actual, 2),
            "pursuit_heading": round(pursuit_heading, 2),
            "along_track_distance": round(along_track_distance, 2),
            "path_length": round(path_length, 2),
            "current_position": self.current_position
        }

    def compute_vector_field_guidance(self):
        if self.mission is None:
            return {
                "guidance_mode": "VECTOR_FIELD",
                "status": "waiting_for_mission",
                "desired_heading": None,
                "path_heading": None,
                "field_strength": None,
                "convergence_gain": None,
                "cross_track_error": None
            }

        waypoints = self.mission.get("waypoints", [])

        if len(waypoints) < 2:
            return {
                "guidance_mode": "VECTOR_FIELD",
                "status": "need_at_least_two_waypoints",
                "desired_heading": None,
                "path_heading": None,
                "field_strength": None,
                "convergence_gain": None,
                "cross_track_error": None
            }

        start_wp = waypoints[self.active_segment]
        end_wp = waypoints[min(self.active_segment + 1, len(waypoints) - 1)]

        end_local = self.latlon_to_local_meters(start_wp, end_wp)
        current_local = self.latlon_to_local_meters(
            start_wp,
            self.current_position
        )

        path_vector = end_local
        path_length = self.vector_norm(path_vector)

        if path_length < 1e-6:
            return {
                "guidance_mode": "VECTOR_FIELD",
                "status": "invalid_path",
                "desired_heading": None,
                "path_heading": None,
                "field_strength": None,
                "convergence_gain": None,
                "cross_track_error": None
            }

        path_unit = [v / path_length for v in path_vector]

        along_track_distance = self.dot_product(current_local, path_unit)

        projection = [
            along_track_distance * path_unit[0],
            along_track_distance * path_unit[1],
            along_track_distance * path_unit[2]
        ]

        error_vector = [
            current_local[0] - projection[0],
            current_local[1] - projection[1],
            current_local[2] - projection[2]
        ]

        cross_track_error = self.vector_norm(error_vector)

        convergence_gain = 0.08

        guidance_vector = [
            path_unit[0] - convergence_gain * error_vector[0],
            path_unit[1] - convergence_gain * error_vector[1],
            path_unit[2] - convergence_gain * error_vector[2]
        ]

        guidance_norm = self.vector_norm(guidance_vector)

        if guidance_norm > 1e-6:
            guidance_unit = [v / guidance_norm for v in guidance_vector]
        else:
            guidance_unit = path_unit

        desired_heading = math.degrees(
            math.atan2(guidance_unit[1], guidance_unit[0])
        )

        path_heading = math.degrees(
            math.atan2(path_unit[1], path_unit[0])
        )

        return {
            "guidance_mode": "VECTOR_FIELD",
            "status": "active",
            "active_segment": self.active_segment + 1,
            "desired_heading": round(desired_heading, 2),
            "path_heading": round(path_heading, 2),
            "cross_track_error": round(cross_track_error, 2),
            "along_track_distance": round(along_track_distance, 2),
            "path_length": round(path_length, 2),
            "convergence_gain": convergence_gain,
            "field_strength": round(guidance_norm, 3),
            "current_position": self.current_position
        }

    def publish_guidance_output(self):
        if self.guidance_mode == "DIRECT_WAYPOINT":
            guidance = self.compute_direct_waypoint_guidance()

        elif self.guidance_mode == "LOS_GUIDANCE":
            guidance = self.compute_los_guidance()

        elif self.guidance_mode == "PURE_PURSUIT":
            guidance = self.compute_pure_pursuit_guidance()

        elif self.guidance_mode == "VECTOR_FIELD":
            guidance = self.compute_vector_field_guidance()

        else:
            guidance = {
                "guidance_mode": self.guidance_mode,
                "status": "mode_not_implemented_yet"
            }

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