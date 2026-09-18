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
            "topic", "/riemannian_motion_policy/real_pose"  # <----- change this to your desired topic
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

        self.x_data_axis.set_xlabel("time [s]")
        self.y_data_axis.set_xlabel("time [s]")
        self.z_data_axis.set_xlabel("time [s]")
        self.rx_data_axis.set_xlabel("time [s]")
        self.ry_data_axis.set_xlabel("time [s]")
        self.rz_data_axis.set_xlabel("time [s]")

        self.x_data_axis.set_ylabel("x position [m]")
        self.y_data_axis.set_ylabel("y position [m]")
        self.z_data_axis.set_ylabel("z position [m]")
        self.rx_data_axis.set_ylabel("rx orientation [rad]")
        self.ry_data_axis.set_ylabel("ry orientation [rad]")
        self.rz_data_axis.set_ylabel("rz orientation [rad]")

        for ax in [
            self.x_data_axis,
            self.y_data_axis,
            self.z_data_axis,
            self.rx_data_axis,
            self.ry_data_axis,
            self.rz_data_axis
        ]:
            ax.yaxis.tick_right()

        self.x_data_axis.legend(loc="upper right")
        self.y_data_axis.legend(loc="upper right")
        self.z_data_axis.legend(loc="upper right")
        self.rx_data_axis.legend(loc="upper right")
        self.ry_data_axis.legend(loc="upper right")
        self.rz_data_axis.legend(loc="upper right")

        self.timer = self.create_timer(0.05, self.update_plot)

        self.start_time = None


    def real_callback(self, message):
        if len(message.data) != 6:
            self.get_logger().warning("Expected 6 pose values")
            return
        
        time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = time
        self.real_time.append(time - self.start_time)
        self.real_values.append(list(message.data))

    def desired_callback(self, message):
        if len(message.data) != 6:
            self.get_logger().warning("Expected 6 desired pose values")
            return
            
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