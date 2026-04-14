#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Trigger
# from geometry_msgs.srv import Pose


class OffboardControlService(Node):
    """Node for controlling a vehicle in offboard mode using ROS2 services."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_service')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create ROS2 services
        self.arm_service = self.create_service(Trigger, 'arm', self.arm_callback)
        self.disarm_service = self.create_service(Trigger, 'disarm', self.disarm_callback)
        self.takeoff_service = self.create_service(Trigger, 'takeoff', self.takeoff_callback)
        self.land_service = self.create_service(Trigger, 'land', self.land_callback)
        self.engage_offboard_service = self.create_service(Trigger, 'engage_offboard', self.engage_offboard_callback)

        # Create a timer to publish control heartbeat signal
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Offboard control service node initialized. Services available:')
        self.get_logger().info('  - /offboard_control_takeoff_service/arm')
        self.get_logger().info('  - /offboard_control_takeoff_service/disarm')
        self.get_logger().info('  - /offboard_control_takeoff_service/takeoff')
        self.get_logger().info('  - /offboard_control_takeoff_service/land')
        self.get_logger().info('  - /offboard_control_takeoff_service/engage_offboard')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm_callback(self, request, response):
        """Service callback for arm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        response.success = True
        response.message = 'Arm command sent successfully'
        return response

    def disarm_callback(self, request, response):
        """Service callback for disarm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        response.success = True
        response.message = 'Disarm command sent successfully'
        return response

    def engage_offboard_callback(self, request, response):
        """Service callback for engage offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')
        response.success = True
        response.message = 'Switched to offboard mode'
        return response

    def takeoff_callback(self, request, response):
        """Service callback for takeoff command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param4=0.0, param7=5.0)
        self.get_logger().info('Takeoff command sent')
        response.success = True
        response.message = 'Takeoff command sent successfully'
        return response

    def land_callback(self, request, response):
        """Service callback for land command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')
        response.success = True
        response.message = 'Land command sent successfully'
        return response

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer - publish heartbeat signal."""
        self.publish_offboard_control_heartbeat_signal()
        
        # Print vehicle status for monitoring
        self.get_logger().debug(f"Vehicle armed status: {self.vehicle_status.arming_state}")
        self.get_logger().debug(f"Vehicle local position (z): {self.vehicle_local_position.z}")


def main(args=None) -> None:
    print('Starting offboard control service node...')
    rclpy.init(args=args)
    offboard_control_service = OffboardControlService()
    rclpy.spin(offboard_control_service)
    offboard_control_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
