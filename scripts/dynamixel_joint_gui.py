#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Manual logical-angle control for the mannequin head bridge."""

import rospy
from std_msgs.msg import Float64

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk


class DynamixelJointGUI:
    def __init__(self):
        rospy.init_node('dynamixel_joint_gui', anonymous=True)

        self.command_topic = rospy.get_param(
            '~command_topic', '/hair_task_manager/head_rotation/command')
        self.feedback_topic = rospy.get_param(
            '~feedback_topic', '/hair_task_manager/head_rotation/feedback')
        self.minimum = float(rospy.get_param('~minimum_head_rad', -3.14))
        self.maximum = float(rospy.get_param('~maximum_head_rad', 3.14))
        self._pending_send = None
        self._latest_feedback = None
        self.pub = rospy.Publisher(
            self.command_topic, Float64, queue_size=1)
        self.feedback_sub = rospy.Subscriber(
            self.feedback_topic, Float64, self._feedback_cb, queue_size=1)

        # GUI
        self.root = tk.Tk()
        self.root.title("Dynamixel Body Joint Control")
        self.root.geometry("400x190")

        # Current position label
        tk.Label(self.root, text="body_joint position (rad):").pack(pady=5)

        # Slider
        self.slider = tk.Scale(
            self.root,
            from_=self.minimum,
            to=self.maximum,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=350,
            command=self.on_slider_change
        )
        self.slider.set(0.0)
        self.slider.pack(pady=10)

        # Position display
        self.pos_label = tk.Label(self.root, text="Position: 0.00 rad")
        self.pos_label.pack(pady=5)

        self.feedback_label = tk.Label(
            self.root, text="Measured: waiting for feedback")
        self.feedback_label.pack(pady=2)

        # Home button
        tk.Button(self.root, text="Home (0.0)", command=self.go_home).pack(pady=5)

    def on_slider_change(self, value):
        pos = float(value)
        self.pos_label.config(text=f"Position: {pos:.2f} rad")
        self.send_trajectory(pos)

    def send_trajectory(self, position):
        # Slider callbacks can arrive much faster than a serial Dynamixel bus
        # should be commanded. Debounce while dragging and publish only the
        # most recent logical angle through the safety bridge.
        if self._pending_send is not None:
            self.root.after_cancel(self._pending_send)
        self._pending_send = self.root.after(
            75, lambda: self._publish_position(position))

    def _publish_position(self, position):
        self._pending_send = None
        self.pub.publish(Float64(data=float(position)))

    def _feedback_cb(self, message):
        self._latest_feedback = float(message.data)

    def _refresh_feedback(self):
        if self._latest_feedback is not None:
            self.feedback_label.config(
                text="Measured: {:.3f} rad".format(self._latest_feedback))
        self.root.after(100, self._refresh_feedback)

    def go_home(self):
        self.slider.set(0.0)
        if self._pending_send is not None:
            self.root.after_cancel(self._pending_send)
            self._pending_send = None
        self._publish_position(0.0)

    def run(self):
        self.root.after(100, self._refresh_feedback)
        self.root.mainloop()


def main():
    try:
        gui = DynamixelJointGUI()
        gui.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
