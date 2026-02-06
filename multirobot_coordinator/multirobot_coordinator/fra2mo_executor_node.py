#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Fra2moExecutor(Node):
    """Fra2mo task executor - navigates to detected sectors"""
    
    def __init__(self):
        super().__init__('fra2mo_executor_node')
        
        # Sector centers mapping (coordinate sulla mappa dove sono gli ArUco)
        self.sector_centers = {
            'N': (0.0, 1.5),    # ArUco ID 5 - Settore Nord
            'E': (2.0, 0.0),    # ArUco ID 10 - Settore Est
            'S': (0.0, -1.5),   # ArUco ID 15 - Settore Sud
            'W': (-2.0, 0.0),   # ArUco ID 20 - Settore Ovest
        }
        
        # Current state
        self.current_sector = None
        self.is_navigating = False
        
        # Subscriber to sector commands
        self.sector_sub = self.create_subscription(
            String,
            '/task/sector',
            self.sector_callback,
            10
        )
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Fra2mo Executor started - waiting for sector commands')
        self.get_logger().info('Sector mapping (ArUco world positions):')
        for sector, coords in self.sector_centers.items():
            self.get_logger().info(f'  Sector {sector}: goal = ({coords[0]}, {coords[1]})')
        self.get_logger().info('=' * 60)
    
    def sector_callback(self, msg: String):
        """Handle sector command"""
        sector = msg.data
        
        # Ignore if already busy
        if self.is_navigating:
            self.get_logger().warn(f'Busy! Ignoring sector command: {sector}')
            return
        
        # Check if sector is valid
        if sector not in self.sector_centers:
            self.get_logger().warn(f'Unknown sector: {sector}')
            return
        
        self.current_sector = sector
        x, y = self.sector_centers[sector]
        
        self.get_logger().info(f'[DEBUG SECTOR_CALLBACK] sector={sector}')
        self.get_logger().info(f'[DEBUG SECTOR_CALLBACK] sector_centers[{sector}] = {self.sector_centers[sector]}')
        self.get_logger().info(f'[DEBUG SECTOR_CALLBACK] x={x}, y={y} (after unpacking)')
        self.get_logger().info(f'Received sector command: "{sector}" -> navigating to ({x}, {y})')
        self.navigate_to(x, y)
    
    def navigate_to(self, x: float, y: float):
        """Send navigation goal to Nav2"""
        
        self.get_logger().info(f'[DEBUG NAVIGATE_TO] FUNCTION ENTRY: x={x}, y={y}, type(x)={type(x)}, type(y)={type(y)}')
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'[DEBUG NAVIGATE_TO] BEFORE ASSIGNMENT: x={x}, y={y}')
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        self.get_logger().info(f'[DEBUG NAVIGATE_TO] AFTER ASSIGNMENT: goal x={goal_msg.pose.pose.position.x}, y={goal_msg.pose.pose.position.y}')
        
        # Orientation (facing towards center)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'[DEBUG] Goal message created:')
        self.get_logger().info(f'  Frame: {goal_msg.pose.header.frame_id}')
        self.get_logger().info(f'  Position: ({goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}, {goal_msg.pose.pose.position.z})')
        self.get_logger().info(f'  Orientation: (x={goal_msg.pose.pose.orientation.x}, y={goal_msg.pose.pose.orientation.y}, z={goal_msg.pose.pose.orientation.z}, w={goal_msg.pose.pose.orientation.w})')
        
        # Send goal
        self.is_navigating = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        
        # Log navigation progress
        nav_time = feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9
        self.get_logger().info(
            f'[NAV FEEDBACK] Distance: {feedback.distance_remaining:.2f}m, '
            f'Time: {nav_time:.1f}s, Recoveries: {feedback.number_of_recoveries}'
        )
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            self.is_navigating = False
            return
        
        self.get_logger().info('Goal accepted by Nav2 - navigating...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        self.is_navigating = False
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Successfully reached sector "{self.current_sector}"!')
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
    

def main(args=None):
    rclpy.init(args=args)
    
    executor_node = Fra2moExecutor()
    
    try:
        rclpy.spin(executor_node)
    except KeyboardInterrupt:
        pass
    finally:
        executor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
