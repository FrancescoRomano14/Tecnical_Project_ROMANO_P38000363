#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import String
import time


class TaskAllocator(Node):
    """Task allocation logic based on ArUco marker detection"""
    
    def __init__(self):
        super().__init__('task_allocator_node')
        
        # Parameters
        self.declare_parameter('debounce_time', 3.0)
        self.debounce_time = self.get_parameter('debounce_time').value
        
        # ArUco ID -> Sector mapping
        self.id_to_sector = {
            5: 'N',   # Nord
            10: 'E',  # Est
            15: 'S',  # Sud
            20: 'W'   # Ovest
        }
        
        # Debounce tracking
        self.last_sector = None
        self.last_publish_time = 0.0
        
        # Subscriber to ArUco detections
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/aruco_marker_publisher/markers',
            self.marker_callback,
            10
        )
        
        # Publisher for sector commands
        self.sector_pub = self.create_publisher(
            String,
            '/task/sector',
            10
        )
        
        self.get_logger().info(
            f'Task Allocator started (debounce={self.debounce_time}s)'
        )
    
    def marker_callback(self, msg: MarkerArray):
        """Process detected ArUco markers"""
        if not msg.markers:
            return
        
        # Process only the first detected marker (highest confidence)
        marker = msg.markers[0]
        marker_id = marker.id
        
        # Check if ID is mapped to a sector
        if marker_id not in self.id_to_sector:
            return
        
        sector = self.id_to_sector[marker_id]
        current_time = time.time()
        
        # Log della posizione rilevata (coordinate del marker rispetto alla camera)
        pos = marker.pose.pose.position
        self.get_logger().info(
            f'[DEBUG] ArUco ID {marker_id} rilevato a: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} (camera frame)'
        )
        
        # Pubblica sempre immediatamente - nessun debounce
        # (il debounce è gestito da fra2mo_executor che ignora comandi se busy)
        
        # Publish sector command
        sector_msg = String()
        sector_msg.data = sector
        self.sector_pub.publish(sector_msg)
        
        # Update tracking
        self.last_sector = sector
        self.last_publish_time = current_time
        
        self.get_logger().info(
            f'Detected ID {marker_id} -> sector "{sector}" - PUBBLICATO COMANDO'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
