#!/usr/bin/env python3

# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Hilary Luo (hluo@clearpathrobotics.com)

from math import floor
from threading import Lock, Thread
from time import sleep

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import numpy as np

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.2  # when the robot will go charge
BATTERY_CRITICAL = 0.1  # when the robot will shutdown

x_low = -3
x_high = 3
y_low = -3
y_high = 3


class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')

        self.lock = lock
        self.battery_percent=50.
        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


def main(args=None):
    rclpy.init(args=args)
    print(args)
    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    navigator = TurtleBot4Navigator()
    battery_percent = None
    position_index = 0

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    # Wait for Nav2
    #navigator.waitUntilNav2Active()

    # Undock
    if navigator.getDockedStatus():
        navigator.undock()


    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent
        if (battery_percent is not None):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

            # Check battery charge level
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('Battery critically low. Charge or power down')
                break

            else:
                # maybe use to get the vicon coordinates: navigator.setInitialPose(initial_pose)

                # Navigate to next position
                position = [np.random.uniform(x_low, x_high), np.random.uniform(y_low, y_high)]
                orientation = np.random.uniform(0, 359)
                goal_pose = navigator.getPoseStamped(position,  orientation)
                print("moving to position: ", position, " orientation: ", orientation)
                navigator.startToPose(goal_pose)

    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
