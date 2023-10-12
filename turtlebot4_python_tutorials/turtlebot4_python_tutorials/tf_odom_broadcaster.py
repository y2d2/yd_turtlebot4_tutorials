
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

class OdomBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_map_odom')


        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Need to add this to be able to read the /odom topic from the trutlebot. Its QoS policies seems to be off.
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(Odometry, "odom", self.update_o_bl_tf, qos_profile=qos_profile)
        #self.subscription = self.create_subscription(Odometry, "odom", self.update_o_bl_tf, 10)



        self.T_o_bl = np.eye(4)
        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
    def update_o_bl_tf(self, msg):
        try:
            self.T_o_bl[:-1, -1] = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                                    msg.pose.pose.position.z]
            r = R.from_quat(
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w])
            self.T_o_bl[:-1, :-1] = r.as_matrix()
            self.publish_baslink_frame()
        except ValueError:
            pass



    def publish_baslink_frame(self):
        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.T_o_bl[0, -1]
        t.transform.translation.y = self.T_o_bl[1, -1]
        t.transform.translation.z = self.T_o_bl[2, -1]

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = quaternion_from_euler(0, 0, 0)
        q = R.from_matrix(self.T_o_bl[:-1, :-1]).as_quat()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        print(t)
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)




def main(args=None):
    print(args)
    rclpy.init(args=args)
    node = OdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
