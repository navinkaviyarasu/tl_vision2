
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleOdometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped

class ViconOdometry(Node):
    def __init__(self):

        super().__init__('vicon_bridge')

        mocap_use_description = ParameterDescriptor(description='\nIntended use of the mocap data:\n 1. EKF sensor fusion \n 2. Ground truth reference\n\n')
        self.declare_parameter('mocap_use', 0, mocap_use_description) # 1.EKF Fusion 2.GroundTruth Reference

        self.pose_enu = None
        self.twist_enu = None
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.mocap_gt_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', 10)
        self.mocap_odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.create_subscription(PoseStamped, '/vrpn_mocap/Akira/pose', self.pose_callback, qos_profile)
        self.create_subscription(TwistStamped, '/vrpn_mocap/Akira/twist', self.twist_callback, qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/50.0, self.mocap_pub) #Runs at 50Hz

    def enutonedTransform(self, position_enu, orientation_enu, velocity_enu):
       
        rot_enutoned = np.array([[0, 1, -0],
                                [1, -0, 0],
                                [0, -0, -1]])

    
        position_ned = np.dot(rot_enutoned, position_enu)
        velocity_ned = np.dot(rot_enutoned, velocity_enu)

        rot_enu = R.from_quat(orientation_enu)
        rot_ned = rot_enu*R.from_matrix(rot_enutoned)
        orientation_ned = rot_ned.as_quat()

        return position_ned, orientation_ned, velocity_ned

    def pose_callback(self, pose_enu):

        self.pose_enu = pose_enu

    def twist_callback(self, twist_enu):

        self.twist_enu = twist_enu

    def mocapOdomPublisher(self, position_ned, orientation_ned, velocity_ned):
            
            mocap_msg = VehicleOdometry()

            mocap_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            mocap_msg.pose_frame = 1
            mocap_msg.position = position_ned.astype(np.float32)
            mocap_msg.q[0] = orientation_ned[3].astype(np.float32)
            mocap_msg.q[1] = orientation_ned[0].astype(np.float32)
            mocap_msg.q[2] = orientation_ned[1].astype(np.float32)
            mocap_msg.q[3] = orientation_ned[2].astype(np.float32)
            mocap_msg.velocity_frame = 1
            mocap_msg.velocity = velocity_ned.astype(np.float32)
            mocap_msg.angular_velocity[:] = np.NaN
            # mocap_msg.position_variance = np.NaN
            # mocap_msg.orientation_variance = np.NaN
            # mocap_msg.velocity_variance = np.NaN

            mocap_use = self.get_parameter('mocap_use').value
            if mocap_use == 1:
                self.mocap_odom_pub.publish(mocap_msg)
                self.get_logger().info("Vicon Bridge Active as EKF source")

            elif mocap_use == 2:
                self.mocap_gt_pub.publish(mocap_msg)
                self.get_logger().info("Vicon Bridge Active as Ground Truth reference")

    def tfPublisher(self, position_enu, orientation_enu):

        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "Akira"
        tf_msg.transform.translation.x = position_enu[0]
        tf_msg.transform.translation.y = position_enu[1]
        tf_msg.transform.translation.z = position_enu[2]
        tf_msg.transform.rotation.x = orientation_enu[0]
        tf_msg.transform.rotation.y = orientation_enu[1]
        tf_msg.transform.rotation.z = orientation_enu[2]
        tf_msg.transform.rotation.w = orientation_enu[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    def mocap_pub(self):
        
        if self.pose_enu and self.twist_enu:

            pos = self.pose_enu.pose.position
            orien = self.pose_enu.pose.orientation
            vel = self.twist_enu.twist.linear

            position_enu = np.array([pos.x, pos.y, pos.z])
            orientation_enu = np.array([orien.x, orien.y, orien.z, orien.w])
            velocity_enu = np.array([vel.x, vel.y, vel.z])

            position_ned, orientation_ned, velocity_ned = self.enutonedTransform(
                position_enu, orientation_enu, velocity_enu
            )

            self.mocapOdomPublisher(position_ned, orientation_ned, velocity_ned)
            self.tfPublisher(position_enu, orientation_enu)

def main(args=None):

    rclpy.init(args=args)
    vicon_node = ViconOdometry()

    mocap_use = vicon_node.get_parameter('mocap_use').value

    while mocap_use not in [1,2]:
        vicon_node.get_logger().error("Select the appropriate usage of the mocap odometry data")
        vicon_node.get_logger().error(f"Current Parameters:\n mocap_use: {mocap_use}")
        rclpy.spin_once(vicon_node)
        mocap_use = vicon_node.get_parameter('mocap_use').value

    if mocap_use ==1:
        vicon_node.get_logger().info("\nMocap Odometry data has been selected for drone localization and will be fused with EKF\n")
    
    elif mocap_use == 2:
        vicon_node.get_logger().info("\nMocap Odometry data has been selected for Ground truth reference only \n")
        
    rclpy.spin(vicon_node)
    vicon_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
