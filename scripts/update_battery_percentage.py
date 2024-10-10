#!/usr/bin/env python3
import signal
import sys

from pyrobosim_msgs.msg import RobotState

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from skiros2_world_model.ros.world_model_interface import WorldModelInterface


class BatteryLevelUpdate(Node):
    """Updates the battery level of a robot in the world model."""

    def __init__(self, robot_name: str = "robot", element_name: str = "cora:Robot-14"):
        """Initialize the BatteryLevelUpdate class."""
        super().__init__("battery_level_update_" + robot_name)
        self.wmi = WorldModelInterface(self, "battery_level_update_" + robot_name)
        self.battery_level = 0.0
        self.last_battery_level = 0.0

        self.element_name = element_name
        self.update_change = 1  # If any value differs more than this, update

        topic = f"/{robot_name}/robot_state"
        self.sub = self.create_subscription(RobotState, topic, self._callback, 1)

    def _isclose(self, f1, f2, allowed_error=0.1):
        return abs(f1 - f2) <= allowed_error

    def _update_element(self):
        if not self._isclose(self.last_battery_level, self.battery_level, self.update_change):
            el = self.wmi.get_element(self.element_name)
            el.setProperty("skiros:BatteryPercentage", self.battery_level, datatype="xsd:double")
            self.wmi.update_element(el)
            self.last_battery_level = self.battery_level

    def _callback(self, msg):
        self.battery_level = round(msg.battery_level, 0)

    def run(self):
        """Run the BatteryLevelUpdate class."""
        timer_period = 0.25
        my_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(timer_period, self._update_element, callback_group=my_callback_group)
        rclpy.spin(self)


if __name__ == '__main__':
    # Allows to exit upon ctrl-c
    def signal_handler(signal, frame):
        """Handle the signal interrupt."""
        print("\nProgram exiting gracefully")
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()

    blu = BatteryLevelUpdate()
    blu.run()
