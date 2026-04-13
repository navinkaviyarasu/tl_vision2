import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

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

        self.vehicleStatusSubscriber = self.create_subscription(VehicleStatus,'fmu/out/vehicle_status_v1',self.vehicleStatusCallback,qos_profile_sub)  
        self.vehicleLocalPositionSubscriber = self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position', self.vehicleLocalPositionCallback, qos_profile_sub)

        self.offboardModePublisher = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_profile_pub)
        self.trajectoryPublisher = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_profile_pub)
        self.vehicleCommandPublisher = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', qos_profile_pub)

        #Attribute assignment

        timePeriod = 0.02  # seconds - 0.02s = 20ms = 50Hz

        self.dt = timePeriod
        # self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.navState = None
        # self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.armingState = None
        self.vehicleLocalPosition = None

        self.offboardCounter = 0

        #Trajectory parameters

        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('takeoff_altitude', 5.0)
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.takeoffAltitude = self.get_parameter('takeoff_altitude').value

        # self.waypoints = [
        #     (1.0, 1.0, -self.altitude),
        #     (1.0, -1.0, -self.altitude),
        #     (-1.0, -1.0, -self.altitude),
        #     (-1.0, 1.0, -self.altitude)
        # ]
        self.waypoints = [
            (0.0, 1.0, -self.takeoffAltitude),
            (2.0, 1.0, -self.takeoffAltitude),
            (2.0, -1.0, -self.takeoffAltitude),
            (0.0, -1.0, -self.takeoffAltitude),
            (0.0, 0.0, -self.takeoffAltitude)
        ]

        self.current_waypoint_index = 0
        self.last_waypoint_time = time.time()
        self.square_completed = False
        self.landing_initiated = False

        self.timer = self.create_timer(timePeriod, self.commandLoopCallback)

    # def arm(self):
    #     """Send an arm command to the vehicle."""
    #     self.publish_vehicle_command(
    #         VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    #     self.get_logger().info('Arm command sent')
    
    def land(self):
        """Switch to land mode."""
        vehicleCommandMsg = VehicleCommand()
        vehicleCommandMsg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        vehicleCommandMsg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicleCommandPublisher.publish(vehicleCommandMsg)
        self.get_logger().info("Switching to land mode")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        vehicleCommandMsg = VehicleCommand()
        vehicleCommandMsg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicleCommandMsg.param1 = 0.0
        vehicleCommandMsg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicleCommandPublisher.publish(vehicleCommandMsg)
        self.get_logger().info('Disarm command sent')

    def vehicleLocalPositionCallback(self, msg):
        self.vehicleLocalPosition = msg
 
    def vehicleStatusCallback(self, msg):
        # TODO: handle NED->ENU transformation
        self.navState = msg.nav_state
        self.armingState = msg.arming_state
        self.get_logger().info(f'Viper Status: State: {self.armingState} Mode: {self.navState}')

    def offboardHeartbeatPublisher(self):
        offboardMsg = OffboardControlMode()
        offboardMsg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboardMsg.position=True
        offboardMsg.velocity=False
        offboardMsg.acceleration=False
        self.offboardModePublisher.publish(offboardMsg)

    def takeoff(self, x, y, z):
    #     VEHICLE_CMD_NAV_TAKEOFF, param3 = self.current_yaw, param4 = self.takeoffaltitude in m(amsl)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param3=0.0, param4=self.takeoffAltitude)
        # self.get_logger().info("Takeoff command sent")

        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectoryPublisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
        
        # trajectory_msg = TrajectorySetpoint()
        # waypoint = self.waypoints[self.current_waypoint_index]
        # trajectory_msg.position[0] = waypoint[0]
        # trajectory_msg.position[1] = waypoint[1]
        # trajectory_msg.position[2] = waypoint[2]
        # self.trajectoryPublisher.publish(trajectory_msg)


    def commandLoopCallback(self):

        self.offboardHeartbeatPublisher()

        if (self.armingState == VehicleStatus.ARMING_STATE_ARMED and self.navState != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
        
            vehicleCommandMsg = VehicleCommand()
            vehicleCommandMsg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            vehicleCommandMsg.param1 = 1.0  # custom mode
            vehicleCommandMsg.param2 = 6.0  # offboard mode
            vehicleCommandMsg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicleCommandPublisher.publish(vehicleCommandMsg)

        if (self.offboardCounter==10 and self.armingState == VehicleStatus.ARMING_STATE_ARMED and self.navState == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.takeoff(0.0, 0.0, -self.takeoffAltitude)

        now = time.time()

        # Check if drone has reached takeoff altitude (negative Z in NED coordinates)
        if self.navState == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.vehicleLocalPosition.z <= -self.takeoffAltitude:

            # Execute waypoint trajectory if not yet completed
            if not self.square_completed:
                if now - self.last_waypoint_time > 2.5 and self.current_waypoint_index < len(self.waypoints):
                    trajectory_msg = TrajectorySetpoint()
                    waypoint = self.waypoints[self.current_waypoint_index]
                    trajectory_msg.position[0] = waypoint[0]
                    trajectory_msg.position[1] = waypoint[1]
                    trajectory_msg.position[2] = waypoint[2]
                    self.trajectoryPublisher.publish(trajectory_msg)
                    self.get_logger().info(f"Waypoint {self.current_waypoint_index}: {waypoint}")

                    self.current_waypoint_index += 1
                    self.last_waypoint_time = time.time()

                    # Check if one complete square is done
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.square_completed = True
                        self.get_logger().info("Square trajectory completed. Landing...")

            # Land and disarm after square is completed
            if self.square_completed and not self.landing_initiated:
                self.landing_initiated = True
                self.land()

            # Disarm once landed
            if self.landing_initiated and self.vehicleLocalPosition.z < 1:
                self.disarm()
                self.get_logger().info("Mission complete. Exiting...")
                rclpy.shutdown()

        if self.offboardCounter<11:
            self.offboardCounter+=1 
        
        else:
            print(f"Vehicle in {self.navState} mode: Switch to Offboard Mode")


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        print("[INFO]:KeyboardInterrupt: Shutting down node....")
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()