import sys
import pathlib

import capnp
import rclpy
import tf2_ros
import numpy as np
import ecal.core.core as ecal_core

from rclpy.node import Node
from ecal.core.subscriber import MessageSubscriber
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VioState

current_path = str(pathlib.Path(__file__).parent.resolve())
capnp.add_import_hook([current_path + '/../capnp'])
import odometry3d_capnp as eCALOdometry3d

class ByteSubscriber(MessageSubscriber):
	"""Specialized publisher subscribes to raw bytes
	"""
	def __init__(self, name):
		topic_type = "base:byte"
		super(ByteSubscriber, self).__init__(name, topic_type)
		self.callback = None

	def receive(self, timeout=0):
		""" receive subscriber content with timeout

		:param timeout: receive timeout in ms

		"""
		ret, msg, time = self.c_subscriber.receive(timeout)
		return ret, msg, time

	def set_callback(self, callback):
		""" set callback function for incoming messages

		:param callback: python callback function (f(topic_name, msg, time))

		"""
		self.callback = callback
		self.c_subscriber.set_callback(self._on_receive)

	def rem_callback(self, callback):
		""" remove callback function for incoming messages

		:param callback: python callback function (f(topic_name, msg, time))

		"""
		self.c_subscriber.rem_callback(self._on_receive)
		self.callback = None

	def _on_receive(self, topic_name, msg, time):
		self.callback(topic_name, msg, time)


class RosOdometryPublisher(Node):
    def __init__(self):
        super().__init__('ros_odometry_publisher')
        
        # ROS2 Publisher setup
        self.ros_odom_pub = self.create_publisher(Odometry, '/vio/data', 10)
        self.viostate_pub = self.create_publisher(VioState, '/vision/vio_state', 10)
        
        # TF2 Broadcaster for ROS2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
    def callback(self, topic_name, msg, time):

        # print(msg) #Just data in bytes

        #some message type and callback error, may be coz of conflict between capnpsubscriber and bytesubscriber
        # with eCALOdometry3d.VioState.from_bytes(msg) as VioStatemsg:
        #       print("VioState message:\n", VioStatemsg)

        with eCALOdometry3d.VioPose.from_bytes(msg) as VioPosemsg:
              print("VioPose message:\n", VioPosemsg)
              
        #some message type and callback error, may be coz of conflict between capnpsubscriber and bytesubscriber
        # with eCALOdometry3d.VioLandmark.from_bytes(msg) as VioLandmarkmsg:
        #       print("VioLandmark message:\n", VioLandmarkmsg)

        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:
            # Create and fill Odometry message

            print("Odometry message:\n", odometryMsg)
            # ros_msg = Odometry()
            # # ros_msg.header = Header()
            # ros_msg.header.stamp = self.get_clock().now().to_msg()
            # ros_msg.header.frame_id = "map"
            # ros_msg.child_frame_id = "vio_frame"
            
            # ros_msg.pose.pose.position.x = odometryMsg.pose.position.x
            # ros_msg.pose.pose.position.y = odometryMsg.pose.position.y
            # ros_msg.pose.pose.position.z = odometryMsg.pose.position.z
            # ros_msg.pose.pose.orientation.w = odometryMsg.pose.orientation.w
            # ros_msg.pose.pose.orientation.x = odometryMsg.pose.orientation.x
            # ros_msg.pose.pose.orientation.y = odometryMsg.pose.orientation.y
            # ros_msg.pose.pose.orientation.z = odometryMsg.pose.orientation.z

            # ros_msg.twist.twist.linear.x = odometryMsg.twist.linear.x
            # ros_msg.twist.twist.linear.y = odometryMsg.twist.linear.y
            # ros_msg.twist.twist.linear.z = odometryMsg.twist.linear.z
            # ros_msg.twist.twist.angular.x = odometryMsg.twist.angular.x
            # ros_msg.twist.twist.angular.y = odometryMsg.twist.angular.y
            # ros_msg.twist.twist.angular.z = odometryMsg.twist.angular.z
            
            # # Publish the Odometry message
            # self.ros_odom_pub.publish(ros_msg)
            
            # # Broadcast the Transform
            # tf_pose = ros_msg.pose.pose
            # tf = TransformStamped()
            # # tf.header.stamp = ros_msg.header.stamp
            # # tf.header.frame_id = ros_msg.header.frame_id
            # tf.child_frame_id = ros_msg.child_frame_id
            # tf.transform.translation.x = tf_pose.position.x
            # tf.transform.translation.y = tf_pose.position.y
            # tf.transform.translation.z = tf_pose.position.z
            # tf.transform.rotation.x = tf_pose.orientation.x
            # tf.transform.rotation.y = tf_pose.orientation.y
            # tf.transform.rotation.z = tf_pose.orientation.z
            # tf.transform.rotation.w = tf_pose.orientation.w
            
            # self.tf_broadcaster.sendTransform(tf)
            
            # # Publish the Vision State message
            # vio_state = VioState()
            # # vio_state.header = Header()
            # vio_state.header.stamp = self.get_clock().now().to_msg()
            # vio_state.header.frame_id = "odom"
            # vio_state.vision_failure = odometryMsg.metricVisionFailureLikelihood
            # vio_state.inertial_failure = odometryMsg.metricInertialFailureLikelihood
            # vio_state.failure_drift = odometryMsg.estimatedFailureModeDrift
            # vio_state.vio_failure = odometryMsg.metricFailureVio

            # self.viostate_pub.publish(vio_state)

            # self.get_logger().info("eCAL-ROS Bridge Active")

def main(args=None):
    print(f"eCAL {ecal_core.getversion()} ({ecal_core.getdate()})\n")
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    ecal_core.set_process_state(1, 1, "I feel good")
    rclpy.init(args=args)

    # Create the ROS2 node and start the subscriber
    node = RosOdometryPublisher()

    # Set up eCAL subscriber (bytes subscriber)
    ecal_topic = "S1/vio_odom"
    node.get_logger().info(f"eCAL-ROS bridge subscribe topic: {ecal_topic}")

    sub = ByteSubscriber(ecal_topic)
    sub.set_callback(node.callback)

    # Keep ROS2 node spinning
    rclpy.spin(node)

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
