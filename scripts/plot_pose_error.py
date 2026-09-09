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
            "topic", "/cartesian_impedance_controller/pose_error"
        ).value
        self.samples = deque(maxlen=2000)
        self.values = deque(maxlen=2000)
        self.subscription = self.create_subscription(
            Float64MultiArray, topic, self.error_callback, 10
        )

        self.figure, (self.position_axis, self.orientation_axis) = plt.subplots(2, 1)
        self.position_lines = [
            self.position_axis.plot([], [], label=label)[0]
            for label in ("x", "y", "z")
        ]
        self.orientation_lines = [
            self.orientation_axis.plot([], [], label=label)[0]
            for label in ("rx", "ry", "rz")
        ]
        self.position_axis.set_ylabel("position error")
        self.orientation_axis.set_ylabel("orientation error")
        self.orientation_axis.set_xlabel("sample")
        self.position_axis.legend()
        self.orientation_axis.legend()
        self.timer = self.create_timer(0.05, self.update_plot)

    def error_callback(self, message):
        if len(message.data) != 6:
            self.get_logger().warning("Expected 6 pose-error values")
            return
        self.samples.append(self.samples[-1] + 1 if self.samples else 0)
        self.values.append(list(message.data))

    def update_plot(self):
        if not self.values:
            return
        samples = list(self.samples)
        values = list(self.values)
        for index, line in enumerate(self.position_lines):
            line.set_data(samples, [value[index] for value in values])
        for index, line in enumerate(self.orientation_lines):
            line.set_data(samples, [value[index + 3] for value in values])
        self.position_axis.relim()
        self.position_axis.autoscale_view()
        self.orientation_axis.relim()
        self.orientation_axis.autoscale_view()
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