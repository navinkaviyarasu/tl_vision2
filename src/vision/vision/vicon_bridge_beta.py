
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import scipy.linalg

from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleOdometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped



class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def conjugate(self):
        # Return the conjugate of the quaternion (w, -x, -y, -z)
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __mul__(self, other):
        # Multiply this quaternion by another quaternion
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(w, x, y, z)

    def __repr__(self):
        return f"({self.w}, {self.x}, {self.y}, {self.z})"

# # Define the quaternion for a 90-degree rotation about the Z-axis
# q_Z90 = Quaternion(np.sqrt(2)/2, 0, 0, np.sqrt(2)/2)

# # Define the quaternion for a 180-degree rotation about the X-axis
# q_X180 = Quaternion(0, 1, 0, 0)

# # The combined frame rotation quaternion (ENU to NED)
# q_frame_rotation = q_Z90 * q_X180

# # Define the quaternion representing the orientation in the ENU frame (example)
# q_ENU = Quaternion(1, 2, 3, 4)

# # Compute the NED quaternion using the formula q_NED = q_frame_rotation * q_ENU * q_frame_rotation.conjugate()
# q_NED = q_frame_rotation * q_ENU * q_frame_rotation.conjugate()

# # Print the resulting NED quaternion
# print("q_NED:", q_NED)


class ViconOdometry(Node):
    def __init__(self):

        super().__init__('vicon_bridge')

        mocap_use_description = ParameterDescriptor(description='\nIntended use of the mocap data:\n 1. EKF sensor fusion \n 2. Ground truth reference\n\n')
        self.declare_parameter('mocap_use', 2, mocap_use_description) # 1.EKF Fusion 2.GroundTruth Reference

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

    def quaternion_mulitply(self, q1, q2):
    
        x = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1]
        y = q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2]
        z = q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0]
        w = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]

        orient_ned= np.array([x, y, z, w])
        return orient_ned
    
    def enutonedTransform(self, position_enu, orientation_enu, velocity_enu):
       
        rot_enutoned = np.array([[0, 1, 0],
                                 [1, 0, 0],
                                 [0, 0, -1]])
        
        # orth_check = scipy.linalg.det(rot_enutoned)

        # print(orth_check)

        position_ned = np.dot(rot_enutoned, position_enu)
        velocity_ned = np.dot(rot_enutoned, velocity_enu)
        print(f"Orientation_ENU: {orientation_enu}")

        # Define the quaternion for a 90-degree rotation about the Z-axis
        q_Z90 = Quaternion(np.sqrt(2)/2, 0, 0, np.sqrt(2)/2)

        # Define the quaternion for a 180-degree rotation about the X-axis
        q_X180 = Quaternion(0, 1, 0, 0)

        # The combined frame rotation quaternion (ENU to NED)
        q_frame_rotation = q_Z90 * q_X180

        # Define the quaternion representing the orientation in the ENU frame (example)
        q_ENU = Quaternion(0,0,0,1)

        # Compute the NED quaternion using the formula q_NED = q_frame_rotation * q_ENU * q_frame_rotation.conjugate()
        q_NED = q_frame_rotation * q_ENU * q_frame_rotation.conjugate()

        # Print the resulting NED quaternion
        print("q_NED:", q_NED)

        # qx, qy, qz, qw = orientation_enu

        # q_vec_enu = np.array([qx, qy, qz])
        # q_vec_ned = np.dot(rot_enutoned, q_vec_enu)
        # ned_quaternion = np.hstack([q_vec_ned, qw])   
        # print(f"Orientation_NED_A0: {ned_quaternion}")    
        
        # rot_enu = R.from_quat(orientation_enu)
        # ROT_ENU2NED = R.from_matrix(rot_enutoned) 
        # rot_ned = rot_enu * ROT_ENU2NED
        # # print("Test___", R.__len__(rot_enu))
        # q_test = rot_ned.as_quat()
        # orient_euler = rot_ned.as_euler('XYZ', degrees = True)
        # # orientation_ned2 = rot_ned.as_matrix()
        # print(f"q_test: ({q_test[0], q_test[1], q_test[2], q_test[3]})")
        # print(f"Orientation in NED [RPY]: {orient_euler}")
        # # print(f"Orientation_NED_A1: {orientation_ned2}")



        # rot_obj = R.from_matrix(rot_enutoned)
        # # print("___", type(rot_obj))
        # enurot_obj = R.from_quat(orientation_enu)
        # orientation_ned_a2 = (rot_obj*enurot_obj).as_quat()
        # print(f"Orientation_NED_A2: {orientation_ned_a2}")

        # euler_enu = R.from_quat(orientation_enu)
        # e_enu = euler_enu.as_euler('xyz', degrees=True)

        # euler_ned = np.dot(rot_enutoned, e_enu)
        # q_ned = R.from_euler('xyz', euler_ned, degrees=True).as_quat()
        # print(f"Orientation_NED_A3: {q_ned}")

        # print(f'Euler ENU:{e_enu}, EULER NED: {euler_ned}')

        ############################
        # Taking rotation from the matrix, i.e., the rotation will be z=90deg, y=0deg & x=180deg
        # q_rotation_obj = R.from_matrix(rot_enutoned)
        # q_rot = q_rotation_obj.as_quat()
        # print("Implicit rotation q", q_rot)

        # #Taking rotation explicitly and the below mentioned rotation is just z=90deg
        # q_rotation = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
        # print("Explicit rotation q", q_rotation)
        # explicit_orient_ned = self.quaternion_mulitply(q_rotation, orientation_enu)
        # implicit_orient_ned = self.quaternion_mulitply(q_rot, orientation_enu)
        # print(f"Quaternion_NED_explicit: {explicit_orient_ned}")
        # print(f"Quaternion_NED_implicit: {implicit_orient_ned}")

        # print(f"Euler ENU:{np.degrees(e_enu[0]):.2f}, {np.degrees(e_enu[1]):.2f}, {np.degrees(e_enu[2]):.2f}")
        #     #   Euler NED:{euler_ned}")
        # obj_enu = R.from_quat(orientation_enu)
        # obj_ned = obj_enu.apply(rot_enutoned)
        # orientation_ned_a3 = obj_ned.as_quat()
        # print(f"Orientation_NED_A3: {orientation_ned_a3}")

        # enu_obj = R.from_quat(orientation_enu)
        # enu_mat = enu_obj.as_matrix()
        # ned_mat = rot_enutoned@enu_mat@rot_enutoned.T
        # ned_obj = R.from_matrix(ned_mat)
        # orientation_ned_a4 = ned_obj.as_quat()
        # print(f"Orientation_NED_A4: {orientation_ned_a4}")

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
            # print("Odometrt_Q:", mocap_msg.q)
            mocap_use = self.get_parameter('mocap_use').value
            if mocap_use == 1:
                self.mocap_odom_pub.publish(mocap_msg)
                # self.get_logger().info("Vicon Bridge Active as EKF source")

            elif mocap_use == 2:
                self.mocap_gt_pub.publish(mocap_msg)
                # self.get_logger().info("Vicon Bridge Active as Ground Truth reference")

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
            # print(f"ENU Frame:\nPosition: {position_enu}, Orientation: {orientation_enu}, Velocity: {velocity_enu}")
            # print(f"NED Frame:\nPosition: {position_ned}, Orientation: {orientation_ned}, Velocity: {velocity_ned}")
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
