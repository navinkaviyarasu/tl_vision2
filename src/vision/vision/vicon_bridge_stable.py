
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleOdometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy 
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped

class ViconOdometry(Node):
    def __init__(self):

        super().__init__('vicon_bridge')

        mocapUse_description = ParameterDescriptor(description='\nIntended use of the mocap data:\n 1. EKF sensor fusion \n 2. Ground truth reference\n\n')
        viconObjectName_description = ParameterDescriptor(description='\nObject name as provided in the motion capture application\n\n')
        
        self.declare_parameter('mocap_use', 2, mocapUse_description) # 1.EKF Fusion 2.GroundTruth Reference #Default set to GroundTruth Reference
        self.declare_parameter('vicon_object_name', 'Aira', viconObjectName_description)

        self.viconObjectName = self.get_parameter('vicon_object_name').value

        poseTopic = f'/vrpn_mocap/{self.viconObjectName}/pose'
        twistTopic = f'/vrpn_mocap/{self.viconObjectName}/twist'

        self.pose_enu = None
        self.twist_enu = None
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.mocap_gt_pub = self.create_publisher(VehicleOdometry, 'fmu/in/vehicle_mocap_odometry', qos_profile_sensor_data)
        self.mocap_odom_pub = self.create_publisher(VehicleOdometry, 'fmu/in/vehicle_visual_odometry', qos_profile_sensor_data)
        self.create_subscription(PoseStamped, poseTopic, self.pose_callback, qos_profile)
        self.create_subscription(TwistStamped, twistTopic, self.twist_callback, qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/50.0, self.mocap_pub) #Runs at 50Hz

    def enutonedTransform(self, position_enu, orientation_enu, velocity_enu):
       
        rot_enutoned = np.array([[0, 1, 0],
                                [1, 0, 0],
                                [0, 0, -1]])

    
        position_ned = np.dot(rot_enutoned, position_enu)
        velocity_ned = np.dot(rot_enutoned, velocity_enu)

        rot_enu = R.from_quat(orientation_enu)
        rot_flu2frd = R.from_euler('x',180, degrees=True)
        rot_ned = R.from_matrix(rot_enutoned) * rot_enu * rot_flu2frd
        orientation_ned = rot_ned.as_quat()
 
        px, py, pz = position_enu
        vx, vy, vz = velocity_enu
        qx, qy, qz, qw = orientation_enu

        #NOTE: Hack to use Vicon's E as N, and convert the ENU to NED
        # position_ned = np.array([px, -py, -pz])
        # velocity_ned = np.array([vx, -vy, -vz])
        # orientation_ned = np.array([qx, -qy, -qz, qw])

        #NOTE: This method currently not working, requires some work

        #TODO: Update scipy library for scalar position modification on quaternion - scalar first/scalar last

        # rot_enu = R.from_quat(orientation_enu)
        # rot_ned = rot_enu*R.from_matrix(rot_enutoned)
        # orientation_ned = rot_ned.as_quat()

        #NOTE: Alternate Method: To change quaternion orientation from ENU to NED coordinate frame, 
        #but remember drone facing EAST is taken as yaw=0 deg

        # q_vec_enu = np.array([qx, qy, qz])
        # q_vec_ned = np.dot(rot_enutoned, q_vec_enu)
        # orientation_ned = np.hstack([q_vec_ned, qw]) #Quaternion in x,y,z,w order

        # orientation_ned = np.array([qy, qx, -qz, qw])

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
        tf_msg.child_frame_id = self.viconObjectName
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
