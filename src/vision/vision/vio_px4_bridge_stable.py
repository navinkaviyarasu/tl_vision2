#!/usr/bin/env python3

# Future Scope:
# Utilize ros parameters for sensor Type, Direction and Orientation
# Establish tf
# Using namespace for running the script for both the sensor Type
# NOTE: But both can't publish data to /fmu/in/vehicle_visual_odometry topic
# NOTE: Flight controller will handle as a single sensor data only & may lead to issues

import sys
import pathlib

import capnp
import rclpy
import tf2_ros
import numpy as np
import ecal.core.core as ecal_core

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from ecal.core.subscriber import MessageSubscriber
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VioState
from vision.capnp_subscriber import CapnpSubscriber

current_path = str(pathlib.Path(__file__).parent.resolve())
capnp.add_import_hook([current_path + '/../capnp'])
import odometry3d_capnp as eCALOdometry3d

sensorType = None

class OdometryPublisher(Node):

	def __init__(self):

		super().__init__('vio_bridge')

        # Declare and retrieve parameters
		self.declare_parameter("namespace", '')  # Default to empty namespace
		self.declare_parameter("sensor_type", 1)  # Default to VK180Pro
		self.declare_parameter("sensor_direction", 1)  # Default to forward facing
		self.declare_parameter("sensor_orientation", [0.0, -10.0, 0.0])
	

		self.sensorType = self.get_parameter('sensor_type').value
		self.sensorDirection = self.get_parameter('sensor_direction').value
		self.sensorOrientation = np.array(self.get_parameter('sensor_orientation').value)
		self.namespace = self.get_parameter('namespace').value
		self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''

		if self.sensorType not in [1, 2]:
			self.sensorType = 1
		if self.sensorDirection not in [1, 2]:
			self.get_logger().error("Invalid sensor direction! Defaulting to forward facing.")
			self.sensorDirection = 1
		if len(self.sensorOrientation) != 3:
			self.get_logger().error("Invalid sensor orientation! Defaulting to [0.0, -10.0, 0.0].")
			self.sensorOrientation = np.array([0.0, -10.0, 0.0])
        
        # QoS profiles
		qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )
		
		qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

		# Publishers
		self.visualOdometryPUB = self.create_publisher(VehicleOdometry, f'{self.namespace_prefix}/fmu/in/vehicle_visual_odometry', 10)
		self.mocapOdometryPUB = self.create_publisher(VehicleOdometry, f'{self.namespace_prefix}/fmu/in/vehicle_mocap_odometry', 10)
		self.vioStatePUB = self.create_publisher(VioState, f'{self.namespace_prefix}/fmu/in/vio_state', 10)

		# TF broadcaster
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

	# def normalizeToBodyFrame(self, position_ned, orientation_ned, l_velocity_ned):
		# yaw, pitch, roll = self.sensorOrientation
		# r_total = R.from_euler('zyx', [yaw,pitch,roll], degrees=True).as_matrix()
		# norm_pos_ned = np.dot(r_total, position_ned)
		# #NOTE: Is it working as expected?
		# r_ned = R.from_quat(orientation_ned)
		# norm_orient_ned = r_ned*R.from_matrix(r_total)
		# norm_orient_ned = norm_orient_ned.as_quat() #Quaternion in x,y,z,w order
		# norm_lvel_ned = np.dot(r_total, l_velocity_ned)
		# return norm_pos_ned,norm_orient_ned, norm_lvel_ned

	def rotateToBodyFrame(self, orientationNED):
		
		yaw, pitch, roll = self.sensorOrientation
		mountedOrientationOBJ = R.from_euler('zyx', [yaw,pitch,roll], degrees=True).as_matrix()

		# norm_pos_ned = np.dot(r_total, position_ned)

		#NOTE: Is it working as expected?
		sensorOrientationOBJ = R.from_quat(orientationNED)
		rotatedOrientationNED = sensorOrientationOBJ*R.from_matrix(mountedOrientationOBJ)
		rotatedOrientationNED = rotatedOrientationNED.as_quat() #Quaternion in x,y,z,w order

		orientationFinalNED = np.zeros(4)

		#Convert quaternion from x,y,z,w order to w,x,y,z order(PX4 order)
		orientationFinalNED[0] = rotatedOrientationNED[3]
		orientationFinalNED[1] = rotatedOrientationNED[0]
		orientationFinalNED[2] = rotatedOrientationNED[1]
		orientationFinalNED[3] = rotatedOrientationNED[2]

		return orientationFinalNED
	
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

	def viostatePublisher(self, vioState):
		
		self.vioStatePUB.publish(vioState)

	def odometryPublisher(self, positionFinalNED, orientationFinalNED, linearVelocityFinalNED):

		# ned_odom_position = n_pos_ned
		# ned_odom_q = nn_q_ned
		# ned_odom_l_velocity = n_l_vel_ned

		visualOdometry = VehicleOdometry()

		visualOdometry.timestamp = int(self.get_clock().now().nanoseconds/1000)
		# visualOdometry.timestamp_sample = # Time at which the sensor data has been captured. Vilota gives stamp at monotonic time
		visualOdometry.pose_frame = 1
		visualOdometry.position = positionFinalNED.astype(np.float32)
		visualOdometry.q = orientationFinalNED.astype(np.float32) # w, x, y, z order
		visualOdometry.velocity_frame = 1
		visualOdometry.velocity = linearVelocityFinalNED.astype(np.float32)
		visualOdometry.angular_velocity[:] = np.nan #ned_odom_a_velocity
		# visualOdometry.position_variance = np.nan
		# visualOdometry.orientation_variance = np.nan
		# visualOdometry.velocity_variance = np.nan
		# visualOdometry.reset_counter = 
		# visualOdometry.quality = 

		self.visualOdometryPUB.publish(visualOdometry)
		self.get_logger().info("Vision bridge online...")

	def callback(self, topic_name, msg, time):

		with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

			position = odometryMsg.pose.position
			orientation = odometryMsg.pose.orientation
			linearVelocity = odometryMsg.twist.linear


			# position_nwu = np.array([position.x, position.y, position.z])
			# # convert orientation data from VIO to x,y,z,w order for scipy processing
			# orientation_nwu = np.array([orientation.x,
			# 				   orientation.y, orientation.z, orientation.w])
			# l_velocity_nwu = np.array([l_velocity.x, l_velocity.y, l_velocity.z])

			# position_ned, orientation_ned, l_velocity_ned = self.nwutonedTransform(position_nwu,
			# 															  orientation_nwu, l_velocity_nwu)
			
			positionNED = np.array([position.x, position.y, position.z])
			# convert orientation data from VIO to x,y,z,w order for scipy processing
			orientationNED = np.array([orientation.x,
							   orientation.y, orientation.z, orientation.w])
			linearVelocityNED = np.array([linearVelocity.x, linearVelocity.y, linearVelocity.z])

			if (self.sensorType == 1 and self.sensorDirection == 1) or (self.sensorType == 2 and self.sensorDirection == 2): #VK180Pro vision module mounted forward facing & VK180 vision module mounted backward facing

				#Convert Vilota sensor axis to drone axis with respect to mounting direction
				positionNED[0] = -(positionNED[0])
				positionNED[1] = -(positionNED[1])
				positionNED[2] = positionNED[2]

				orientationNED[0] = -(orientationNED[0]) #q_x
				orientationNED[1] = -(orientationNED[1]) #q_y
				orientationNED[2] = (orientationNED[2]) #q_z
				orientationNED[3] = orientationNED[3] #q_w

				linearVelocityNED[0] = -(linearVelocityNED[0])
				linearVelocityNED[1] = -(linearVelocityNED[1])
				linearVelocityNED[2] = linearVelocityNED[2]

			elif (self.sensorType == 1 and self.sensorDirection == 2) or (self.sensorType == 2 and self.sensorDirection == 1): #VK180Pro vision module mounted backward facing & VK180 vision module mounted forward facing 

				positionNED = positionNED
				orientationNED = orientationNED
				linearVelocityNED = linearVelocityNED

			# n_pos_ned, n_q_ned, n_l_vel_ned = self.normalizeToBodyFrame(position_ned,
			# 												   orientation_ned, l_velocity_ned)

			positionFinalNED = positionNED
			orientationFinalNED = self.rotateToBodyFrame(orientationNED)
			linearVelocityFinalNED = linearVelocityNED

			
			vioState = VioState()
			vioState.timestamp = int(self.get_clock().now().nanoseconds/1000)
			vioState.vision_failure = odometryMsg.metricVisionFailureLikelihood
			vioState.inertial_failure = odometryMsg.metricInertialFailureLikelihood
			vioState.failure_drift = odometryMsg.estimatedFailureModeDrift
			vioState.vio_failure = odometryMsg.metricFailureVio
			vioState.reset_counter = odometryMsg.resetCounter

			self.odometryPublisher(positionFinalNED, orientationFinalNED, linearVelocityFinalNED)
			self.viostatePublisher(vioState)
			# self.tfPublisher(position_nwu, orientation_nwu)

def main(args=None):

	rclpy.init(args=args)
	odometryPublisher = OdometryPublisher()
	sensorType = odometryPublisher.get_parameter('sensor_type').value
	print(f"eCAL {ecal_core.getversion()} ({ecal_core.getdate()})\n")
	ecal_core.initialize(sys.argv, "vio_bridge")
	ecal_core.set_process_state(1, 1, "I feel good")	

	while True:			
		if sensorType == 1:
			# print("\nSelected external vision source is VK180Pro")
			visionModule = "VK180Pro"
			eCALtopic = "S1/vio_odom_ned"
		elif sensorType == 2:
			# print("\nSelected external vision source is VK180")
			visionModule = "VK180"
			eCALtopic = "S0/vio_odom_ned"
		else:
			odometryPublisher.get_logger().warn("Invalid sensor type!")
			odometryPublisher.get_logger().info("Defaulting to VK180Pro")
			eCALtopic = "S1/vio_odom_ned"
		
		break
	
	print(f"\neCAL-ROS bridge subscribe topic: {eCALtopic}\n")
	eCALsubscriber = CapnpSubscriber("Odometry3d", eCALtopic)
	eCALsubscriber.set_callback(odometryPublisher.callback)
	try:
		rclpy.spin(odometryPublisher)
	except KeyboardInterrupt:
		print("[INFO]:KeyboardInterrupt: Shutting down node....")
	finally:
		odometryPublisher.destroy_node()
		rclpy.shutdown
	ecal_core.finalize()

if __name__ == "__main__":
	main()
