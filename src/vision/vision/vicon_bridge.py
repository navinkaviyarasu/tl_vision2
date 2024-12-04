
import rclpy
import numpy as np
from rclpy.node import Node

#from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped

class ViconOdometry(Node):
    def __init__(self, mocap_use):

        super().__init__('vicon_bridge')
        self.mocap_use = mocap_use
        self.latest_pose = None
        self.latest_twist = None
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.mocap_gt_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', 10)
        self.mocap_odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.create_subscription(PoseStamped, '/vrpn_mocap/Akira/pose', self.pose_callback, qos_profile)
        self.create_subscription(TwistStamped, '/vrpn_mocap/Akira/twist', self.twist_callback, qos_profile)
        #self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/50.0, self.mocap_pub)


    def pose_callback(self, pose):
        self.latest_pose = pose

    def twist_callback(self, twist):
        self.latest_twist = twist

    def mocap_pub(self):
        
        if self.latest_pose and self.latest_twist:

            pose = self.latest_pose
            twist = self.latest_twist

            #tf_msg = TransformStamped()
            mocap_msg = VehicleOdometry()

            mocap_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            mocap_msg.pose_frame = 1
            mocap_msg.position[0] = (pose.pose.position.x)
            mocap_msg.position[1] = -(pose.pose.position.y)
            mocap_msg.position[2] = -(pose.pose.position.z)
            mocap_msg.q[0] = pose.pose.orientation.w
            mocap_msg.q[1] = (pose.pose.orientation.x)
            mocap_msg.q[2] = -(pose.pose.orientation.y)
            mocap_msg.q[3] = -(pose.pose.orientation.z)
            mocap_msg.velocity_frame = 1
            mocap_msg.velocity[0] = twist.twist.linear.x
            mocap_msg.velocity[1] = -(twist.twist.linear.y)
            mocap_msg.velocity[2] = -(twist.twist.linear.z)
            mocap_msg.angular_velocity[0] = np.NaN
            mocap_msg.angular_velocity[1] = np.NaN
            mocap_msg.angular_velocity[2] = np.NaN
            # mocap_msg.position_variance = np.NaN
            # mocap_msg.orientation_variance = np.NaN
            # mocap_msg.velocity_variance = np.NaN

            #tf_msg.header.stamp = self.get_clock().now().to_msg()
            #tf_msg.header.frame_id = "odom"
            #tf_msg.child_frame_id = "Akira"
            #tf_msg.transform.translation.x = pose.pose.position.x
            #tf_msg.transform.translation.y = pose.pose.position.y
            #tf_msg.transform.translation.z = pose.pose.position.z
            #tf_msg.transform.rotation.w = pose.pose.orientation.w
            #tf_msg.transform.rotation.x = pose.pose.orientation.x
            #tf_msg.transform.rotation.y = pose.pose.orientation.y
            #tf_msg.transform.rotation.z = pose.pose.orientation.z

            #self.tf_broadcaster.sendTransform(tf_msg)

            if self.mocap_use == 1:
                self.mocap_odom_pub.publish(mocap_msg)

            elif self.mocap_use == 2:
                self.mocap_gt_pub.publish(mocap_msg)

            self.get_logger().info("Vicon Bridge Active")

def main(args=None):

    rclpy.init(args=None)
    while True:
        try:
            mocap_use = int(input("Select the intended use of the mocap data:\n 1. EKF sensor fusion \n 2. Ground truth reference\n\n"))
            if mocap_use ==1:
                print("\n Mocap Odometry data has been selected for drone localization and will be fused with EKF\n")
            
            elif mocap_use == 2:
                print("Mocap Odometry data has been selected for Ground truth reference only \n")

            elif mocap_use not in [1,2]:
                raise ValueError("\n\n Select the appropriate usage of the mocap odometry data\n\n")
            
            break
    
        except ValueError as e:
            print(f"Error:{e}")
            
    vicon_node = ViconOdometry(mocap_use)
    rclpy.spin(vicon_node)
    vicon_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
