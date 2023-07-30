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



def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf_map_odom')
        self.ns = self.get_namespace()
        if self.ns == '/':
            self.ns = ''
        else:
            self.ns = self.ns[1:] + '/'
        timer_period = 0.05  # seconds

        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.from_frame="odom"
        self.to_frame="base_link"
        #self.subscription = self.create_subscription(Odometry,'/odom', self.update_odom_tf, 10)



        self.T_o_bl = np.eye(4)
        self.T_m_bl = np.eye(4)
        self.T_m_o = np.eye(4)
        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.timer = self.create_timer(1/10, self.timer_callback)
    def timer_callback(self):
        self.get_T_o_bl()
        self.T_m_o = self.T_m_bl @ np.linalg.inv(self.T_o_bl)
        self.publish_odom_frame()


    def get_T_o_bl(self):
        try:
            t_o_bl = self.tf_buffer.lookup_transform(self.from_frame, self.to_frame, rclpy.time.Time())
            self.T_o_bl[:-1, -1] = [t_o_bl.transform.translation.x, t_o_bl.transform.translation.y,
                                    t_o_bl.transform.translation.z]
            r= R.from_quat(
                [t_o_bl.transform.rotation.x, t_o_bl.transform.rotation.y, t_o_bl.transform.rotation.z,
                 t_o_bl.transform.rotation.w])
            self.T_o_bl[:-1, :-1] = r.as_matrix()
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame} to {self.from_frame}: {ex}')
            return


    def update_odom_tf(self, msg):
        print(msg)
        self.t_o_bl[-1, :-1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.t_o_bl[:-1, :-1] = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.t_m_o = np.linalg.inv(self.t_m_bl) @ self.t_o_bl
        self.publish_odom_frame()

    def publish_odom_frame(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.T_m_o[0, -1]
        t.transform.translation.y = self.T_m_o[1, -1]
        t.transform.translation.z = self.T_m_o[2, -1]

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = quaternion_from_euler(0, 0, 0)
        q = R.from_matrix(self.T_m_o[:-1, :-1]).as_quat()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()