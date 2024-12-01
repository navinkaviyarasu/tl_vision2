#!/usr/bin/env python3

import sys
import capnp
import pathlib
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
#from scipy.spatial.transform import Rotation as R


from px4_msgs.msg import VehicleOdometry, VioState

import ecal.core.core as ecal_core

# Import the Cap'n Proto schema and message
# current_path = str(pathlib.Path(__file__).parent.resolve())
# capnp_schema_path = current_path + '/../src/capnp'
capnp_schema_path = '/home/nk/Workspace/vio_ws/src/vision/src/capnp'
capnp.add_import_hook([capnp_schema_path])

import odometry3d_capnp as eCALOdometry3d

from ecal.core.subscriber import MessageSubscriber

vision_module_type = None

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

class RosOdometryPublisher(rclpy.node.Node):
    def __init__(self, vision_module_type):
        super().__init__('vio_bridge')

        self.vision_module_type = vision_module_type

        # Publishers
        self.vio_odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        #self.mocap_odom_pub = self.create_publisher(VehicleOdometry,'fmu/in/vehicle_mocap_odometry', 10)
        self.viostate_pub = self.create_publisher(VioState, '/fmu/in/vio_state', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def callback(self, topic_name, msg, time):
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

            vio_msg = VehicleOdometry()
            if self.vision_module_type == 1: # For VK180Pro conversion to FRD world-fixed frame with arbitrary heading reference

                vio_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
                # vio_msg.timestamp_sample = 
                vio_msg.pose_frame = 1
                vio_msg.position[0] = -(odometryMsg.pose.position.x)
                vio_msg.position[1] = odometryMsg.pose.position.y
                vio_msg.position[2] = -(odometryMsg.pose.position.z)
                vio_msg.q[0] = odometryMsg.pose.orientation.w
                vio_msg.q[1] = -(odometryMsg.pose.orientation.x)
                vio_msg.q[2] = odometryMsg.pose.orientation.y
                vio_msg.q[3] = -(odometryMsg.pose.orientation.z)
                vio_msg.velocity_frame = 1
                vio_msg.velocity[0] = -(odometryMsg.twist.linear.x)
                vio_msg.velocity[1] = odometryMsg.twist.linear.y
                vio_msg.velocity[2] = -(odometryMsg.twist.linear.z)
                vio_msg.angular_velocity[0] = np.NaN
                vio_msg.angular_velocity[0] = np.NaN
                vio_msg.angular_velocity[0] = np.NaN
                # vio_msg.position_variance = np.NaN
                # vio_msg.orientation_variance = np.NaN
                # vio_msg.velocity_variance = np.NaN

            elif self.vision_module_type == 2: # For VK180 conversion to FRD world-fixed frame with arbitrary heading reference

                vio_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
                # vio_msg.timestamp_sample = 
                vio_msg.pose_frame = 1
                vio_msg.position[0] = odometryMsg.pose.position.x
                vio_msg.position[1] = -(odometryMsg.pose.position.y)
                vio_msg.position[2] = -(odometryMsg.pose.position.z)
                vio_msg.q[0] = odometryMsg.pose.orientation.w
                vio_msg.q[1] = odometryMsg.pose.orientation.x
                vio_msg.q[2] = -(odometryMsg.pose.orientation.y)
                vio_msg.q[3] = -(odometryMsg.pose.orientation.z)
                vio_msg.velocity_frame = 1
                vio_msg.velocity[0] = odometryMsg.twist.linear.x
                vio_msg.velocity[1] = -(odometryMsg.twist.linear.y)
                vio_msg.velocity[2] = -(odometryMsg.twist.linear.z)
                vio_msg.angular_velocity[0] = np.NaN
                vio_msg.angular_velocity[1] = np.NaN
                vio_msg.angular_velocity[2] = np.NaN
                # vio_msg.position_variance = np.NaN
                # vio_msg.orientation_variance = np.NaN
                # vio_msg.velocity_variance = np.NaN

            vio_msg.reset_counter = odometryMsg.resetCounter
            vio_msg.quality = 0

            viostate_msg = VioState()
            viostate_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            viostate_msg.vision_failure = odometryMsg.metricVisionFailureLikelihood
            viostate_msg.inertial_failure = odometryMsg.metricInertialFailureLikelihood
            viostate_msg.failure_drift = odometryMsg.estimatedFailureModeDrift
            viostate_msg.vio_failure = odometryMsg.metricFailureVio
            viostate_msg.reset_counter = odometryMsg.resetCounter
            #print(viostate_msg)

            # conversion = R.from_quat(vio_msg.q)
            # euler_angles = conversion.as_euler('xyz', degrees = True)
            # print("Euler angles in  degrees:", euler_angles)

            self.vio_odom_pub.publish(vio_msg)
            self.viostate_pub.publish(viostate_msg)
            # self.mocap_odom_pub.publish(vio_msg)


            # tf_pose = vio_msg.pose.pose
            # tf = TransformStamped()
            # tf.header.stamp = self.get_clock().now().to_msg()
            # tf.header.frame_id = vio_msg.header.frame_id
            # tf.child_frame_id = vio_msg.child_frame_id
            # tf.transform.translation.x = tf_pose.position.x
            # tf.transform.translation.y = tf_pose.position.y
            # tf.transform.translation.z = tf_pose.position.z
            # tf.transform.rotation.x = tf_pose.orientation.x
            # tf.transform.rotation.y = tf_pose.orientation.y
            # tf.transform.rotation.z = tf_pose.orientation.z
            # tf.transform.rotation.w = tf_pose.orientation.w

            # self.tf_broadcaster.sendTransform(tf)

            # Create and publish the VioState message
            # vio_state = VioState()
            # vio_state.header.seq = odometryMsg.header.seq
            # vio_state.header.stamp = self.get_clock().now().to_msg()
            # vio_state.header.frame_id = "odom"
            # vio_state.vision_failure = odometryMsg.metricVisionFailureLikelihood
            # vio_state.inertial_failure = odometryMsg.metricInertialFailureLikelihood
            # vio_state.failure_drift = odometryMsg.estimatedFailureModeDrift
            # vio_state.vio_failure = odometryMsg.metricFailureVio

            # self.viostate_pub.publish(vio_state)

            self.get_logger().info("VIO Bridge Active")

def main():
    global vision_module_type
    print(f"eCAL {ecal_core.getversion()} ({ecal_core.getdate()})\n")
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    ecal_core.set_process_state(1, 1, "I feel good")

    vision_module_type = None

    while True:
        try:
            vision_module_type = int(input("Select the external vision source type:\n 1. VK180Pro \n 2. VK180\n\n"))
            if vision_module_type == 1:
                print("\nSelected external vision source is VK180Pro")
                ecal_topic = "S1/vio_odom"

            elif vision_module_type == 2:
               print("\nSelected external vision source is VK180")
               ecal_topic = "S0/vio_odom"

            if vision_module_type not in [1,2]:
                raise ValueError("\n\nSource type doesn't exist!\n\n")
            break

        except ValueError as e:
            print(f"Error:{e}")

    rclpy.init()
    ros_odometry_publisher = RosOdometryPublisher(vision_module_type)
    
    print(f"eCAL-ROS bridge subscribe topic: {ecal_topic}")
    sub = ByteSubscriber(ecal_topic)
    sub.set_callback(ros_odometry_publisher.callback)
    rclpy.spin(ros_odometry_publisher)

    ecal_core.finalize()

if __name__ == "__main__":
    main()
