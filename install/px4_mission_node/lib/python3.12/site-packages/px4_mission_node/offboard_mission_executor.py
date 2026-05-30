import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)


class OffboardMissionExecutor(Node):

    def __init__(self):
        super().__init__("offboard_mission_executor")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_profile
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos_profile
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )

        self.progress_pub = self.create_publisher(
            String,
            "mission_progress",
            10
        )

        self.mission_sub = self.create_subscription(
            String,
            "mission_upload",
            self.mission_callback,
            10
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0
        self.current_local_position = [0.0, 0.0, 0.0]

        self.mission_waypoints = []
        self.local_setpoints = []
        self.active_index = 0

        self.current_setpoint = [0.0, 0.0, -10.0]
        self.mission_active = False
        self.mission_state = "Idle"

        self.get_logger().info("Dynamic Offboard Mission Executor started")
        self.get_logger().info("Waiting for mission_upload messages...")

    def mission_callback(self, msg):
        try:
            mission = json.loads(msg.data)
            waypoints = mission.get("waypoints", [])

            if len(waypoints) == 0:
                self.get_logger().warn("Received empty mission")
                return

            self.mission_waypoints = waypoints
            self.local_setpoints = []

            for index, wp in enumerate(waypoints):
                x = index * 10.0
                y = index * 5.0
                z = -float(wp.get("alt", 10.0))
                self.local_setpoints.append([x, y, z])

            self.active_index = 0
            self.current_setpoint = self.local_setpoints[0]
            self.mission_active = True
            self.mission_state = "Running"
            self.counter = 0

            self.get_logger().info("Mission loaded into offboard executor")
            self.get_logger().info(f"Waypoint count: {len(self.local_setpoints)}")

            self.publish_progress()

        except Exception as e:
            self.get_logger().error(f"Mission parse failed: {e}")

    def local_position_callback(self, msg):
        self.current_local_position = [
            float(msg.x),
            float(msg.y),
            float(msg.z)
        ]

    def publish_progress(self):
        total = len(self.local_setpoints)

        if total == 0:
            progress = 0
            active_waypoint = 0
        else:
            active_waypoint = self.active_index + 1
            progress = round((active_waypoint / total) * 100)

        payload = {
            "mission_state": self.mission_state,
            "active_waypoint": active_waypoint,
            "total_waypoints": total,
            "progress_percent": progress
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.progress_pub.publish(msg)

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

        self.command_pub.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [
            float(self.current_setpoint[0]),
            float(self.current_setpoint[1]),
            float(self.current_setpoint[2])
        ]
        msg.yaw = 0.0

        self.trajectory_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def set_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )

    def update_waypoint_progress(self):
        if not self.mission_active:
            return

        if self.counter % 80 == 0:
            self.get_logger().info(
                f"Simulated reached waypoint {self.active_index + 1}"
            )

            self.publish_progress()

            if self.active_index < len(self.local_setpoints) - 1:
                self.active_index += 1
                self.current_setpoint = self.local_setpoints[self.active_index]

                self.get_logger().info(
                    f"Switching to waypoint {self.active_index + 1}: {self.current_setpoint}"
                )

                self.publish_progress()
            else:
                self.get_logger().info("Mission complete. Holding final waypoint.")
                self.mission_active = False
                self.mission_state = "Completed"
                self.publish_progress()

    def timer_callback(self):
        self.publish_offboard_mode()
        self.publish_trajectory_setpoint()

        if self.counter == 20:
            self.set_offboard_mode()
            self.arm()
            self.get_logger().info("OFFBOARD mode requested and ARM command sent")

        self.update_waypoint_progress()

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = OffboardMissionExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()