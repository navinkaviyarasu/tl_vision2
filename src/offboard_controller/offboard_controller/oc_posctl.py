import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare and retrieve the namespace parameter
        self.declare_parameter('namespace', '')  # Default to empty namespace
        self.namespace = self.get_parameter('namespace').value
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''
        
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

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.namespace_prefix}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'{self.namespace_prefix}/fmu/in/offboard_control_mode', qos_profile_pub)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'{self.namespace_prefix}/fmu/in/trajectory_setpoint', qos_profile_pub)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

        self.waypoints = [
            (1.0, 1.0, -self.altitude),
            (1.0, -1.0, -self.altitude),
            (-1.0, -1.0, -self.altitude),
            (-1.0, 1.0, -self.altitude)
        ]

        self.current_waypoint_index = 0
        self.last_waypoint_time = time.time()
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        now = time.time()        
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            # trajectory_msg = TrajectorySetpoint()
            # trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            # trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            # trajectory_msg.position[2] = -self.altitude
            # self.publisher_trajectory.publish(trajectory_msg)

            # self.theta = self.theta + self.omega * self.dt

            if now - self.last_waypoint_time > 2.5:
                trajectory_msg = TrajectorySetpoint()
                waypoint = self.waypoints[self.current_waypoint_index]
                trajectory_msg.position[0] = waypoint[0]
                trajectory_msg.position[1] = waypoint[1]
                trajectory_msg.position[2] = waypoint[2]
                self.publisher_trajectory.publish(trajectory_msg)

                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
                self.last_waypoint_time = time.time()


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