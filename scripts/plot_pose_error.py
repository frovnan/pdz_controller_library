#!/usr/bin/env python3

from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PoseErrorPlotter(Node):
    def __init__(self):
        super().__init__("pose_error_plotter")
        topic = self.declare_parameter(
            "topic", "/cartesian_impedance_controller/real_pose"  # <----- change this to your desired topic
        ).value
        desired_topic = self.declare_parameter(
            "desired_topic", "/user_input_client/desired_pose"  # <----- change this to your desired topic
        ).value
        self.real_samples = deque(maxlen=2000)
        self.real_values = deque(maxlen=2000)
        self.desired_samples = deque(maxlen=2000)
        self.desired_values = deque(maxlen=2000)
        self.real_subscription = self.create_subscription(
            Float64MultiArray, topic, self.real_callback, 10
        )
        self.desired_subscription = self.create_subscription(
            Float64MultiArray, desired_topic, self.desired_callback, 10
        )
        self.figure, (self.x_data_axis, self.y_data_axis, self.z_data_axis, self.rx_data_axis, self.ry_data_axis, self.rz_data_axis) = plt.subplots(6, 1)
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
        self.x_data_axis.set_ylabel("x data")
        self.y_data_axis.set_ylabel("y data")
        self.z_data_axis.set_ylabel("z data")
        self.rx_data_axis.set_ylabel("rx data")
        self.ry_data_axis.set_ylabel("ry data")
        self.rz_data_axis.set_ylabel("rz data")
        self.x_data_axis.set_xlabel("sample")
        self.y_data_axis.set_xlabel("sample")
        self.z_data_axis.set_xlabel("sample")
        self.rx_data_axis.set_xlabel("sample")
        self.ry_data_axis.set_xlabel("sample")
        self.rz_data_axis.set_xlabel("sample")
        self.x_data_axis.legend()
        self.y_data_axis.legend()
        self.z_data_axis.legend()
        self.rx_data_axis.legend()
        self.ry_data_axis.legend()
        self.rz_data_axis.legend()
        self.timer = self.create_timer(0.05, self.update_plot)

    def real_callback(self, message):
        if len(message.data) != 6:
            self.get_logger().warning("Expected 6 pose values")
            return
        self.real_samples.append(self.real_samples[-1] + 1 if self.real_samples else 0)
        self.real_values.append(list(message.data))

    def desired_callback(self, message):
        if len(message.data) != 6:
            self.get_logger().warning("Expected 6 desired pose values")
            return
        self.desired_samples.append(self.desired_samples[-1] + 1 if self.desired_samples else 0)
        self.desired_values.append(list(message.data))

    def update_plot(self):
        if not self.real_values:
            return
        real_samples = list(self.real_samples)
        desired_samples = list(self.desired_samples)
        real_values = list(self.real_values)
        desired_values = list(self.desired_values)

        self.x_lines[0].set_data(
            real_samples,
            [v[0] for v in real_values]
        )

        self.x_lines[1].set_data(
            desired_samples,
            [v[0] for v in desired_values]
        )
        self.y_lines[0].set_data(
            real_samples,
            [v[1] for v in real_values]
        )

        self.y_lines[1].set_data(
            desired_samples,
            [v[1] for v in desired_values]
        )
        self.z_lines[0].set_data(
            real_samples,
            [v[2] for v in real_values]
        )
        self.z_lines[1].set_data(
            desired_samples,
            [v[2] for v in desired_values]
        )
        self.rx_lines[0].set_data(
            real_samples,
            [v[3] for v in real_values]
        )
        self.rx_lines[1].set_data(
            desired_samples,
            [v[3] for v in desired_values]
        )
        self.ry_lines[0].set_data(
            real_samples,
            [v[4] for v in real_values]
        )
        self.ry_lines[1].set_data(
            desired_samples,
            [v[4] for v in desired_values]
        )
        self.rz_lines[0].set_data(
            real_samples,
            [v[5] for v in real_values]
        )
        self.rz_lines[1].set_data(
            desired_samples,
            [v[5] for v in desired_values]
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
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


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