#!/usr/bin/env python3

import sys
import pathlib

import capnp
import rclpy
import tf2_ros
import numpy as np
import ecal.core.core as ecal_core

from ecal.core.subscriber import MessageSubscriber
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VioState

current_path = str(pathlib.Path(__file__).parent.resolve())
capnp.add_import_hook([current_path + '/../capnp'])
import odometry3d_capnp as eCALOdometry3d

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