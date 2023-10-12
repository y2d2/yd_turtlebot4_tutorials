#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from vicon_receiver.msg import Position

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf_map_odom')
        self.ns = self.get_namespace()
        vicon_topic = '/vicon/tb1' + self.ns
        print(vicon_topic)
        if self.ns == '/':
            self.ns = ''
        else:
            self.ns = self.ns[1:] + '/'


        # self.subscription = self.create_subscription(Position, '/vicon/tb1/tb1', self.update_odom_tf, 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(Odometry, "odom", self.update_o_bl_tf, qos_profile=qos_profile)



        self.T_o_bl = np.eye(4)
        self.T_m_bl = np.eye(4)
        self.T_m_o = np.eye(4)
        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message

    def update_odom_tf(self, msg):
        pass

    def update_o_bl_tf(self,msg):
        print(msg)




def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
