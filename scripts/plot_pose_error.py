#!/usr/bin/env python3

from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import subprocess
import re
import time
from pathlib import Path

result = subprocess.check_output(
    ["ros2", "control", "list_controllers"],
    text=True
)
result = re.sub(r'\x1b\[[0-9;]*m', '', result) # filter out ANSI color escape codes for line.split() to work cleanly

controller_name = "placeholder"

for line in result.splitlines():
    name = line.split()[0]
    state = line.split()[-1]
    if state == "active" and name != "joint_state_broadcaster":
        controller_name = name
        if controller_name == "placeholder":
            raise RuntimeError("No active controller found.")
        break

class PoseErrorPlotter(Node):
    def __init__(self):
        super().__init__("pose_error_plotter")

        topic = self.declare_parameter(
            "topic", f"/{controller_name}/real_pose"
        ).value
        desired_topic = self.declare_parameter(
            "desired_topic", "/user_input_client/desired_pose"
        ).value

        self.real_time = deque(maxlen=2000)
        self.real_values = deque(maxlen=2000)
        self.desired_time = deque(maxlen=2000)
        self.desired_values = deque(maxlen=2000)

        self.real_subscription = self.create_subscription(
            Float64MultiArray, topic, self.real_callback, 10
        )
        self.desired_subscription = self.create_subscription(
            Float64MultiArray, desired_topic, self.desired_callback, 10
        )

        self.figure, (self.x_data_axis, self.y_data_axis, self.z_data_axis, self.rx_data_axis, self.ry_data_axis, self.rz_data_axis, self.position_error_norm_data_axis, self.geodesic_data_axis) = plt.subplots(8, 1)

        self.x_lines = [
            self.x_data_axis.plot([], [], label=label)[0]
            for label in ("x", "x_des")
        ]
        self.y_lines = [
            self.y_data_axis.plot([], [], label=label)[0]
            for label in ("y", "y_des")
        ]
        self.z_lines = [
            self.z_data_axis.plot([], [], label=label)[0]
            for label in ("z", "z_des")
        ]
        self.rx_lines = [
            self.rx_data_axis.plot([], [], label=label)[0]
            for label in ("rx", "rx_des")
        ]
        self.ry_lines = [
            self.ry_data_axis.plot([], [], label=label)[0]
            for label in ("ry", "ry_des")
        ]
        self.rz_lines = [
            self.rz_data_axis.plot([], [], label=label)[0]
            for label in ("rz", "rz_des")
        ]
        self.position_error_line = [
            self.position_error_norm_data_axis.plot([], label="position_error_norm")[0]
        ]
        self.geodesic_line = [
            self.geodesic_data_axis.plot([], label="geodesic_error")[0]
        ]

        self.x_data_axis.set_xlabel("time [s]")
        self.y_data_axis.set_xlabel("time [s]")
        self.z_data_axis.set_xlabel("time [s]")
        self.rx_data_axis.set_xlabel("time [s]")
        self.ry_data_axis.set_xlabel("time [s]")
        self.rz_data_axis.set_xlabel("time [s]")
        self.position_error_norm_data_axis.set_xlabel("time [s]")
        self.geodesic_data_axis.set_xlabel("time [s]")

        self.x_data_axis.set_ylabel("x [m]")
        self.y_data_axis.set_ylabel("y [m]")
        self.z_data_axis.set_ylabel("z [m]")
        self.rx_data_axis.set_ylabel("rx [rad]")
        self.ry_data_axis.set_ylabel("ry [rad]")
        self.rz_data_axis.set_ylabel("rz [rad]")
        self.position_error_norm_data_axis.set_ylabel("position error norm [m]")
        self.geodesic_data_axis.set_ylabel("geodesic metric error [rad]")

        for ax in [
            self.x_data_axis,
            self.y_data_axis,
            self.z_data_axis,
            self.rx_data_axis,
            self.ry_data_axis,
            self.rz_data_axis,
            self.position_error_norm_data_axis,
            self.geodesic_data_axis
        ]:
            ax.yaxis.tick_right()

        self.x_data_axis.legend(loc="upper right")
        self.y_data_axis.legend(loc="upper right")
        self.z_data_axis.legend(loc="upper right")
        self.rx_data_axis.legend(loc="upper right")
        self.ry_data_axis.legend(loc="upper right")
        self.rz_data_axis.legend(loc="upper right")
        self.position_error_norm_data_axis.legend(loc="upper right")
        self.geodesic_data_axis.legend(loc="upper right")

        self.timer = self.create_timer(0.05, self.update_plot)

        self.start_time = None

        self.duration = 50.0
        self.saved = False


    def real_callback(self, message):
        #if len(message.data) != 6:
        #    self.get_logger().warning("Expected 6 pose values")
        #    return
        
        time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = time
        self.real_time.append(time - self.start_time)
        self.real_values.append(list(message.data))

    def desired_callback(self, message):
        #if len(message.data) != 6:
        #    self.get_logger().warning("Expected 6 desired pose values")
        #    return
            
        time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = time
        self.desired_time.append(time - self.start_time)
        self.desired_values.append(list(message.data))

    def update_plot(self):
        if not self.real_values:
            return
        real_time = list(self.real_time)
        real_values = list(self.real_values)
        desired_time = list(self.desired_time)
        desired_values = list(self.desired_values)

        self.x_lines[0].set_data(
            real_time,
            [v[0] for v in real_values]
        )

        self.x_lines[1].set_data(
            desired_time,
            [v[0] for v in desired_values]
        )
        self.y_lines[0].set_data(
            real_time,
            [v[1] for v in real_values]
        )

        self.y_lines[1].set_data(
            desired_time,
            [v[1] for v in desired_values]
        )
        self.z_lines[0].set_data(
            real_time,
            [v[2] for v in real_values]
        )
        self.z_lines[1].set_data(
            desired_time,
            [v[2] for v in desired_values]
        )
        self.rx_lines[0].set_data(
            real_time,
            [v[3] for v in real_values]
        )
        self.rx_lines[1].set_data(
            desired_time,
            [v[3] for v in desired_values]
        )
        self.ry_lines[0].set_data(
            real_time,
            [v[4] for v in real_values]
        )
        self.ry_lines[1].set_data(
            desired_time,
            [v[4] for v in desired_values]
        )
        self.rz_lines[0].set_data(
            real_time,
            [v[5] for v in real_values]
        )
        self.rz_lines[1].set_data(
            desired_time,
            [v[5] for v in desired_values]
        )
        self.position_error_line[0].set_data(
            real_time,
            [v[6] for v in real_values]
        )
        self.geodesic_line[0].set_data(
            real_time,
            [v[7] for v in real_values]
        )

        self.x_data_axis.relim()
        self.x_data_axis.autoscale_view()
        self.y_data_axis.relim()
        self.y_data_axis.autoscale_view()
        self.z_data_axis.relim()
        self.z_data_axis.autoscale_view()
        self.rx_data_axis.relim()
        self.rx_data_axis.autoscale_view()
        self.ry_data_axis.relim()
        self.ry_data_axis.autoscale_view()
        self.rz_data_axis.relim()
        self.rz_data_axis.autoscale_view()
        self.position_error_norm_data_axis.relim()
        self.position_error_norm_data_axis.autoscale_view()
        self.geodesic_data_axis.relim()
        self.geodesic_data_axis.autoscale_view()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        self.figure.suptitle(controller_name)

        if (
            not self.saved
            and time.time() - self.start_time > self.duration
        ):
            results_dir = Path.home() / "franka_ros2_ws" / "src" / "pdz_controller_library" / "results"
            results_dir.mkdir(parents=True, exist_ok=True)
            filename = results_dir / f"{controller_name}.png"
            self.figure.savefig(filename, dpi=300)

            self.saved = True

            self.get_logger().info(
                f"Saved plot to {filename}"
            )
            plt.close(self.figure)


def main(args=None):
    rclpy.init(args=args)
    plotter = PoseErrorPlotter()
    plt.ion()
    plt.show(block=False)
    try:
        rclpy.spin(plotter)
    finally:
        plotter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()