import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleGlobalPosition,
    BatteryStatus,
    VehicleStatus
)


class PX4TelemetryPublisher(Node):

    def __init__(self):
        super().__init__("px4_telemetry_publisher")

        self.latest_battery = "--"
        self.latest_nav_state = "--"
        self.latest_arming_state = "--"
        self.latest_failsafe = False

        self.latest_latitude = 47.3977
        self.latest_longitude = 8.5456
        self.latest_global_altitude = "--"

        self.telemetry_pub = self.create_publisher(
            String,
            "telemetry_data",
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            qos_profile
        )

        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.global_position_callback,
            qos_profile
        )

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            self.battery_callback,
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            qos_profile
        )

        self.get_logger().info("PX4 real telemetry publisher started")

    def nav_state_label(self, nav_state):
        labels = {
            0: "MANUAL",
            1: "ALTCTL",
            2: "POSCTL",
            3: "AUTO_MISSION",
            4: "AUTO_LOITER",
            5: "AUTO_RTL",
            6: "ACRO",
            10: "OFFBOARD",
            14: "AUTO_TAKEOFF",
            15: "AUTO_LAND",
            18: "AUTO_LOITER"
        }

        return labels.get(nav_state, f"UNKNOWN ({nav_state})")

    def arming_state_label(self, arming_state):
        labels = {
            1: "DISARMED",
            2: "ARMED"
        }

        return labels.get(arming_state, f"UNKNOWN ({arming_state})")

    def battery_callback(self, msg):
        if msg.remaining >= 0:
            self.latest_battery = round(msg.remaining * 100, 2)

    def status_callback(self, msg):
        self.latest_nav_state = self.nav_state_label(int(msg.nav_state))
        self.latest_arming_state = self.arming_state_label(int(msg.arming_state))
        self.latest_failsafe = bool(msg.failsafe)

    def global_position_callback(self, msg):
        self.latest_latitude = round(float(msg.lat), 7)
        self.latest_longitude = round(float(msg.lon), 7)
        self.latest_global_altitude = round(float(msg.alt), 2)

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
            "battery": self.latest_battery,
            "flight_mode": self.latest_nav_state,
            "arming_state": self.latest_arming_state,
            "failsafe": self.latest_failsafe,
            "latitude": self.latest_latitude,
            "longitude": self.latest_longitude,
            "global_altitude": self.latest_global_altitude
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


if __name__ == "__main__":
    main()