#!/usr/bin/env python3

import csv
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8


class PoseErrorLogger(Node):

    def __init__(self):
        super().__init__("pose_error_logger")

        self.declare_parameter("controller_name")
        self.controller_name = (self.get_parameter("controller_name").value)

        real_topic = self.declare_parameter(
            "topic", f"/{self.controller_name}/real_pose"
        ).value

        desired_topic = self.declare_parameter(
            "desired_topic", "/user_input_client/desired_pose"
        ).value

        trajectory_topic = self.declare_parameter(
            "trajectory_topic", "/user_input_client/trajectory"
        ).value

        self.duration = 60.0
        self.start_ros_time = None
        self.start_wall_time = None
        self.saved = False
        self.trajectory_type = None

        # Store raw messages and their timestamps in memory.
        self.real_rows = []
        self.desired_rows = []

        self.real_subscription = self.create_subscription(
            Float64MultiArray,
            real_topic,
            self.real_callback,
            10,
        )

        self.desired_subscription = self.create_subscription(
            Float64MultiArray,
            desired_topic,
            self.desired_callback,
            10,
        )

        self.trajectory_subscription = self.create_subscription(
            UInt8,
            trajectory_topic,
            self.trajectory_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.check_duration)

        self.get_logger().info(
            f"Logging measured poses from {real_topic}"
        )
        self.get_logger().info(
            f"Logging desired poses from {desired_topic}"
        )

    def relative_ros_time(self):
        return (
            self.get_clock().now().nanoseconds * 1e-9
            - self.start_ros_time
        )

    def real_callback(self, message):
        if self.start_ros_time is None:
            return

        values = list(message.data)

        # Expected format:
        # x, y, z, rx, ry, rz, position_error_norm, geodesic_error
        self.real_rows.append({
            "time_s": self.relative_ros_time(),
            "x": values[0] if len(values) > 0 else "",
            "y": values[1] if len(values) > 1 else "",
            "z": values[2] if len(values) > 2 else "",
            "rx": values[3] if len(values) > 3 else "",
            "ry": values[4] if len(values) > 4 else "",
            "rz": values[5] if len(values) > 5 else "",
            "position_error_norm": values[6] if len(values) > 6 else "",
            "geodesic_error": values[7] if len(values) > 7 else "",
        })

    def desired_callback(self, message):
        if self.start_ros_time is None:
            self.start_ros_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )
            self.start_wall_time = time.monotonic()

        values = list(message.data)

        # Expected format: x, y, z, rx, ry, rz
        self.desired_rows.append({
            "time_s": self.relative_ros_time(),
            "x": values[0] if len(values) > 0 else "",
            "y": values[1] if len(values) > 1 else "",
            "z": values[2] if len(values) > 2 else "",
            "rx": values[3] if len(values) > 3 else "",
            "ry": values[4] if len(values) > 4 else "",
            "rz": values[5] if len(values) > 5 else "",
        })

    def trajectory_callback(self, message):
        self.trajectory_type = int(message.data)

    def check_duration(self):
        if self.start_wall_time is None or self.saved:
            return

        if time.monotonic() - self.start_wall_time >= self.duration:
            self.save_csv()

    def save_csv(self):
        if self.saved:
            return

        results_dir = (
            Path.home()
            / "franka_ros2_ws"
            / "src"
            / "pdz_controller_library"
            / "results"
            / self.controller_name
        )
        results_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = f"{self.controller_name}_{self.trajectory_type}_{timestamp}"

        real_fields = [
            "time_s", "x", "y", "z", "rx", "ry", "rz",
            "position_error_norm", "geodesic_error",
        ]
        desired_fields = [
            "time_s", "x", "y", "z", "rx", "ry", "rz",
        ]

        real_path = results_dir / f"{prefix}_real.csv"
        desired_path = results_dir / f"{prefix}_desired.csv"

        with real_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=real_fields)
            writer.writeheader()
            writer.writerows(self.real_rows)

        with desired_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=desired_fields)
            writer.writeheader()
            writer.writerows(self.desired_rows)

        self.saved = True

        self.get_logger().info(
            f"Saved {len(self.real_rows)} measured samples to {real_path}"
        )
        self.get_logger().info(
            f"Saved {len(self.desired_rows)} desired samples to {desired_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    print("Starting pose logger.")
    logger = PoseErrorLogger()

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        # Save collected data even if the node is stopped early.
        logger.save_csv()
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()