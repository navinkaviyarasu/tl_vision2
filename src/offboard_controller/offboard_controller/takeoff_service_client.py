#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class TakeoffServiceClient(Node):
    """Client node to control drone takeoff using ROS2 services."""

    def __init__(self) -> None:
        super().__init__('takeoff_service_client')
        
        # Create service clients
        self.arm_client = self.create_client(Trigger, '/offboard_control_takeoff_service/arm')
        self.engage_offboard_client = self.create_client(Trigger, '/offboard_control_takeoff_service/engage_offboard')
        self.takeoff_client = self.create_client(Trigger, '/offboard_control_takeoff_service/takeoff')
        self.land_client = self.create_client(Trigger, '/offboard_control_takeoff_service/land')
        self.disarm_client = self.create_client(Trigger, '/offboard_control_takeoff_service/disarm')

        # Wait for services to be available
        self.wait_for_services()

    def wait_for_services(self):
        """Wait for all services to be available."""
        services = [
            (self.arm_client, '/offboard_control_takeoff_service/arm'),
            (self.engage_offboard_client, '/offboard_control_takeoff_service/engage_offboard'),
            (self.takeoff_client, '/offboard_control_takeoff_service/takeoff'),
            (self.land_client, '/offboard_control_takeoff_service/land'),
            (self.disarm_client, '/offboard_control_takeoff_service/disarm'),
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {service_name} not available, waiting...')
        
        self.get_logger().info('All services are available!')

    def arm_drone(self):
        """Send arm command via service."""
        self.get_logger().info('Sending arm command...')
        request = Trigger.Request()
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Arm response: {future.result().message}')
            return True
        else:
            self.get_logger().error('Failed to call arm service')
            return False

    def engage_offboard(self):
        """Switch to offboard mode via service."""
        self.get_logger().info('Engaging offboard mode...')
        request = Trigger.Request()
        future = self.engage_offboard_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Offboard response: {future.result().message}')
            return True
        else:
            self.get_logger().error('Failed to call engage_offboard service')
            return False

    def takeoff(self):
        """Send takeoff command via service."""
        self.get_logger().info('Sending takeoff command...')
        request = Trigger.Request()
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Takeoff response: {future.result().message}')
            return True
        else:
            self.get_logger().error('Failed to call takeoff service')
            return False

    def land(self):
        """Send land command via service."""
        self.get_logger().info('Sending land command...')
        request = Trigger.Request()
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Land response: {future.result().message}')
            return True
        else:
            self.get_logger().error('Failed to call land service')
            return False

    def disarm_drone(self):
        """Send disarm command via service."""
        self.get_logger().info('Sending disarm command...')
        request = Trigger.Request()
        future = self.disarm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Disarm response: {future.result().message}')
            return True
        else:
            self.get_logger().error('Failed to call disarm service')
            return False

    def execute_takeoff_sequence(self):
        """Execute a complete takeoff sequence using services."""
        self.get_logger().info('Starting takeoff sequence...')
        
        # Step 1: Arm the drone
        if not self.arm_drone():
            self.get_logger().error('Failed to arm drone')
            return
        
        # Wait a bit for the arm command to take effect
        time.sleep(1.0)
        
        # Step 2: Engage offboard mode
        if not self.engage_offboard():
            self.get_logger().error('Failed to engage offboard mode')
            return
        
        # Wait a bit for mode switch
        time.sleep(1.0)
        
        # Step 3: Takeoff
        if not self.takeoff():
            self.get_logger().error('Failed to takeoff')
            return
        
        self.get_logger().info('Takeoff sequence completed successfully!')


def main(args=None) -> None:
    print('Starting takeoff service client...')
    rclpy.init(args=args)
    client = TakeoffServiceClient()
    
    # Execute the takeoff sequence
    client.execute_takeoff_sequence()
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")

