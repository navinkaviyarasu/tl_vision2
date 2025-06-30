#!/usr/bin/env python3

# Future Scope:
# Utilize ros parameters for sensor Type, Direction and Orientation
# Establish tf
# Using namespace for running the script for both the sensor Type
# NOTE: But both can't publish data to /fmu/in/vehicle_visual_odometry topic
# NOTE: Flight controller will handle as a single sensor data only & may lead to issues
# TODO: Error in orientation received at FC, Debugging required!
# WIP: Utilize ros parameters for sensor Type, Direction and Orientation

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

sensorType = None
sensorDirection = None

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

class OdometryPublisher(Node):

	def __init__(self, sensorOrientation, sensorType, sensorDirection):

		super().__init__('vio_bridge')
		self.sensorOrientation = sensorOrientation
		self.sensorType = sensorType
		self.sensorDirection = sensorDirection

		# Publishers
		self.vio_odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
		self.mocap_odom_pub = self.create_publisher(VehicleOdometry,'fmu/in/vehicle_mocap_odometry', 10)
		self.viostate_pub = self.create_publisher(VioState, '/vision/vio_state', 10)

		# TF broadcaster
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

	def normalizeToBodyFrame(self, position_ned, orientation_ned, l_velocity_ned):
		
		yaw, pitch, roll = self.sensorOrientation
		r_total = R.from_euler('zyx', [yaw,pitch,roll], degrees=True).as_matrix()

		norm_pos_ned = np.dot(r_total, position_ned)

		#NOTE: Is it working as expected?
		r_ned = R.from_quat(orientation_ned)
		norm_orient_ned = r_ned*R.from_matrix(r_total)
		norm_orient_ned = norm_orient_ned.as_quat() #Quaternion in x,y,z,w order

		norm_lvel_ned = np.dot(r_total, l_velocity_ned)

		return norm_pos_ned,norm_orient_ned, norm_lvel_ned

	def normalizeToBodyFrame_final(self, orientation_ned):
		
		yaw, pitch, roll = self.sensorOrientation
		r_total = R.from_euler('zyx', [yaw,pitch,roll], degrees=True).as_matrix()

		# norm_pos_ned = np.dot(r_total, position_ned)

		#NOTE: Is it working as expected?
		r_ned = R.from_quat(orientation_ned)
		norm_orient_ned = r_ned*R.from_matrix(r_total)
		norm_orient_ned = norm_orient_ned.as_quat() #Quaternion in x,y,z,w order

		# norm_lvel_ned = np.dot(r_total, l_velocity_ned)

		return norm_orient_ned
	
	# def nwutoenuTransform(self, position_nwu, orientation_nwu):

		nwu_to_enu_matrix = np.array([[0,1,0],
								[-1,0,0],
								[0,0,1]])

		position_enu = np.dot(nwu_to_enu_matrix, position_nwu)

		r_nwu = R.from_quat(orientation_nwu)
		r_enu = r_nwu*R.from_matrix(nwu_to_enu_matrix)
		orientation_enu = r_enu.as_quat()

		return position_enu, orientation_enu

	# def nwutonedTransform(self, position_nwu, orientation_nwu, l_velocity_nwu):
		
		nwu_to_ned_matrix = np.array([[1,0,0],
					  [0,-1,0],
					  [0,0,-1]])
		
		position_ned = np.dot(nwu_to_ned_matrix, position_nwu)

		l_velocity_ned = np.dot(nwu_to_ned_matrix, l_velocity_nwu)

		r_nwu = R.from_quat(orientation_nwu) #Input quaternion in x,y,z,w order
		r_ned = r_nwu*R.from_matrix(nwu_to_ned_matrix)
		orientation_ned = r_ned.as_quat() # Quaternion in x,y,z,w order
		
		return position_ned, orientation_ned, l_velocity_ned

	# def tfPublisher(self, position_nwu, orientation_nwu):

		position_enu, orientation_enu = self.nwutoenuTransform(position_nwu, orientation_nwu)
		#Note orientation_enu is in x,y,z,w order
		tf = TransformStamped()
		tf.header.stamp = self.get_clock().now().to_msg()
		tf.header.frame_id = "odom"
		tf.child_frame_id = "vk180"
		tf.transform.translation.x = position_enu[0]
		tf.transform.translation.y = position_enu[1]
		tf.transform.translation.z = position_enu[2]
		tf.transform.rotation.x = orientation_enu[0]
		tf.transform.rotation.y = orientation_enu[1]
		tf.transform.rotation.z = orientation_enu[2]
		tf.transform.rotation.w = orientation_enu[3]

		self.tf_broadcaster.sendTransform(tf)

	def viostatePublisher(self, viostate_msg):
		
		self.viostate_pub.publish(viostate_msg)

	def odometryPublisher(self, n_pos_ned,nn_q_ned, n_l_vel_ned):

		ned_odom_position = n_pos_ned
		ned_odom_q = nn_q_ned
		ned_odom_l_velocity = n_l_vel_ned

		vio_msg = VehicleOdometry()

		vio_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
		# vio_msg.timestamp_sample = # Time at which the sensor data has been captured. Vilota gives stamp at monotonic time
		vio_msg.pose_frame = 1
		vio_msg.position = ned_odom_position.astype(np.float32)
		vio_msg.q = ned_odom_q.astype(np.float32) # w, x, y, z order
		vio_msg.velocity_frame = 1
		vio_msg.velocity = ned_odom_l_velocity.astype(np.float32)
		vio_msg.angular_velocity[:] = np.nan #ned_odom_a_velocity
		# vio_msg.position_variance = np.nan
		# vio_msg.orientation_variance = np.nan
		# vio_msg.velocity_variance = np.nan
		# vio_msg.reset_counter = 
		# vio_msg.quality = 

		self.vio_odom_pub.publish(vio_msg)
		self.get_logger().info("Vision bridge online...")

	def callback(self, topic_name, msg, time):

		with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

			position = odometryMsg.pose.position
			orientation = odometryMsg.pose.orientation
			l_velocity = odometryMsg.twist.linear

			# position_nwu = np.array([position.x, position.y, position.z])
			# # convert orientation data from VIO to x,y,z,w order for scipy processing
			# orientation_nwu = np.array([orientation.x,
			# 				   orientation.y, orientation.z, orientation.w])
			# l_velocity_nwu = np.array([l_velocity.x, l_velocity.y, l_velocity.z])

			# position_ned, orientation_ned, l_velocity_ned = self.nwutonedTransform(position_nwu,
			# 															  orientation_nwu, l_velocity_nwu)
			
			position_ned = np.array([position.x, position.y, position.z])
			# convert orientation data from VIO to x,y,z,w order for scipy processing
			orientation_ned = np.array([orientation.x,
							   orientation.y, orientation.z, orientation.w])
			l_velocity_ned = np.array([l_velocity.x, l_velocity.y, l_velocity.z])

			if self.sensorType == 1 and self.sensorDirection ==1: #VK180Pro vision module mounted forward facing

				position_ned[0] = -(position_ned[0])
				position_ned[1] = -(position_ned[1])
				position_ned[2] = position_ned[2]

				orientation_ned[0] = -(orientation_ned[0]) #q_x
				orientation_ned[1] = -(orientation_ned[1]) #q_y
				orientation_ned[2] = (orientation_ned[2]) #q_z
				orientation_ned[3] = orientation_ned[3] #q_w

				l_velocity_ned[0] = -(l_velocity_ned[0])
				l_velocity_ned[1] = -(l_velocity_ned[1])
				l_velocity_ned[2] = l_velocity_ned[2]

			elif self.sensorType == 1 and self.sensorDirection ==2: #VK180Pro vision module mounted backward facing

				position_ned = position_ned
				orientation_ned = orientation_ned
				l_velocity_ned = l_velocity_ned

			elif self.sensorType == 2 and self.sensorDirection == 1: #VK180 vision module mounted forward facing 
				position_ned = position_ned
				orientation_ned = orientation_ned
				l_velocity_ned = l_velocity_ned

			elif self.sensorType == 2 and self.sensorDirection == 2: #VK180 vision module mounted backward facing
				position_ned[0] = -(position_ned[0])
				position_ned[1] = -(position_ned[1])
				position_ned[2] = position_ned[2]

				orientation_ned[0] = -(orientation_ned[0]) #q_x
				orientation_ned[1] = -(orientation_ned[1]) #q_y
				orientation_ned[2] = (orientation_ned[2]) #q_z
				orientation_ned[3] = orientation_ned[3] #q_w

				l_velocity_ned[0] = -(l_velocity_ned[0])
				l_velocity_ned[1] = -(l_velocity_ned[1])
				l_velocity_ned[2] = l_velocity_ned[2]

			# n_pos_ned, n_q_ned, n_l_vel_ned = self.normalizeToBodyFrame(position_ned,
			# 												   orientation_ned, l_velocity_ned)

			n_pos_ned = position_ned
			n_q_ned = self.normalizeToBodyFrame_final(orientation_ned)
			n_l_vel_ned = l_velocity_ned

			
			#convert quaternion to w,x,y,z order
			nn_q_ned = np.zeros(4)
			nn_q_ned[0] = n_q_ned[3]
			nn_q_ned[1] = n_q_ned[0]
			nn_q_ned[2] = n_q_ned[1]
			nn_q_ned[3] = n_q_ned[2]

			viostate_msg = VioState()
			viostate_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
			viostate_msg.vision_failure = odometryMsg.metricVisionFailureLikelihood
			viostate_msg.inertial_failure = odometryMsg.metricInertialFailureLikelihood
			viostate_msg.failure_drift = odometryMsg.estimatedFailureModeDrift
			viostate_msg.vio_failure = odometryMsg.metricFailureVio
			viostate_msg.reset_counter = odometryMsg.resetCounter

			self.odometryPublisher(n_pos_ned,nn_q_ned, n_l_vel_ned)
			self.viostatePublisher(viostate_msg)
			# self.tfPublisher(position_nwu, orientation_nwu)

def main():

	print(f"eCAL {ecal_core.getversion()} ({ecal_core.getdate()})\n")
	ecal_core.initialize(sys.argv, "test_odometry_sub")
	ecal_core.set_process_state(1, 1, "I feel good")

	# parameter to select the type/make of the vision module, heading direction of 
	# the vision module, and Orientation of the vision module with respect to the 
	# FRD body frame of the drone
	# as the coordinate from orientation of the vk180 & vk180pro are different
	# TODO: To be replaced by ros_parameters

	# while True:
	# 	try:
	# 		sensorType = int(input("Select the external vision source type:\n 1. VK180Pro \n 2. VK180\n\n"))
	# 		if sensorType == 1:
	# 			# print("\nSelected external vision source is VK180Pro")
	# 			visionModule = "VK180Pro"
	# 			ecal_topic = "S1/vio_odom_ned"

	# 		elif sensorType == 2:
	# 			# print("\nSelected external vision source is VK180")
	# 			visionModule = "VK180"
	# 			ecal_topic = "S0/vio_odom_ned"

	# 		if sensorType not in [1,2]:
	# 			raise ValueError("Sensor type doesn't exist! Choose from the available option!")
			
	# 		break

	# 	except ValueError as e:
	# 		print(f"\nError:{e}")
		
	# while True:
	# 	try:
	# 		sensorDirection = int(input(f"\nSelect the heading direction of {visionModule}\n\n1.Forward_Facing\n2.Backward_Facing\n\n"))
	# 		if sensorDirection == 1:
	# 			pass

	# 		elif sensorDirection == 2:
	# 			pass

	# 		if sensorDirection not in [1,2]:
	# 			raise ValueError("Heading direction not applicable!Choose from the available options!")
			
	# 		break

	# 	except ValueError as e:
	# 		print(f"\nError:{e}")

	# # parameter to normalizeToBodyFrame the mounting orientation of the vision module
	# print("""\nEnter the mount orientation of the vision module with respect
	#    to the body frame FRD in order of yaw, pitch and roll (in degrees): """)

	# yaw, pitch, roll = map(float, input().split())
	# sensorOrientation = np.array([yaw,pitch,roll]) #Orientation of the sensor WRT to FRD frame

	#Hardcoded values for sensorType, sensorDirection and sensorOrientation 

	sensorType =1
	sensorDirection =1
	sensorOrientation =np.array([0, -10, 0])
	ecal_topic = "S1/vio_odom_ned"

	# TODO: Output to screen - consolidated sensor information - Type, Orientation & Direction
	print(f"\neCAL-ROS bridge subscribe topic: {ecal_topic}")
	rclpy.init()
	odometry_Publisher = OdometryPublisher(sensorOrientation, sensorType, sensorDirection)
	sub = ByteSubscriber(ecal_topic)
	sub.set_callback(odometry_Publisher.callback)
	rclpy.spin(odometry_Publisher)

	ecal_core.finalize()

if __name__ == "__main__":
	main()
