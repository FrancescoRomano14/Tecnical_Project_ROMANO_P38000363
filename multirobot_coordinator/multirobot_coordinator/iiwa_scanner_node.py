#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class IiwaScanner(Node):
    def __init__(self):
        super().__init__('iiwa_scanner_node')
        
        # Parameters
        self.declare_parameter('scan_period', 30.0)
        self.declare_parameter('update_rate', 10.0)  # Aumentato da 2.0 a 10.0 Hz
        self.declare_parameter('startup_timeout', 5.0)
        
        self.scan_period = self.get_parameter('scan_period').value
        self.update_rate = self.get_parameter('update_rate').value
        self.startup_timeout = self.get_parameter('startup_timeout').value
        
        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/iiwa/iiwa_arm_controller/joint_trajectory',
            10
        )
        
        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/iiwa/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Internal state
        self.current_joints = None
        self.target_angle = 0.0
        self.initialized = False
        
        # Velocità angolare per rotazione continua (rad/s) - aumentata del 50%
        self.angular_velocity = ((2 * math.pi) / self.scan_period) * 1.5
        
        # Oscillation mode: avanti-indietro invece di rotazione continua
        self.scan_direction = 1  # 1 = avanti, -1 = indietro
        # Limiti effettivi del joint iiwa (da URDF): ±2.96 rad
        # Uso limiti leggermente inferiori per sicurezza
        self.min_angle = -2.9  # ~-166°
        self.max_angle = 2.9   # ~+166°
        
        # Timer for continuous scanning
        self.scan_timer = None
        
        # Subscribe to sector commands to detect when marker is found
        self.sector_sub = self.create_subscription(
            String,
            '/task/sector',
            self.sector_callback,
            10
        )
        
        # Timer di timeout per iniziare comunque
        self.startup_timer = self.create_timer(
            self.startup_timeout,
            self.force_start
        )
        
        self.get_logger().info(
            f'Iiwa Scanner started (period={self.scan_period}s, '
            f'rate={self.update_rate}Hz, timeout={self.startup_timeout}s)'
        )
        
    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from feedback"""
        if self.current_joints is None:
            self.current_joints = list(msg.position)
            self.target_angle = msg.position[0]
            
            if not self.initialized:
                self.start_scanning()
        else:
            self.current_joints = list(msg.position)
    
    def force_start(self):
        """Forza l'inizio se joint_states non arriva"""
        if not self.initialized:
            self.get_logger().warn(
                f'No joint states after {self.startup_timeout}s, starting with defaults'
            )
            self.current_joints = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
            self.target_angle = 0.0
            self.start_scanning()
        
        self.startup_timer.cancel()
    
    def sector_callback(self, msg: String):
        """Handle sector detection - just log, don't stop rotation"""
        self.get_logger().info(f'[SECTOR] Marker detected: {msg.data} (continuing rotation)')
    
    def start_scanning(self):
        """Inizializza e avvia lo scanning"""
        self.initialized = True
        self.scan_timer = self.create_timer(
            1.0 / self.update_rate,
            self.scan_callback
        )
        self.get_logger().info('Scanner initialized - starting rotation')
        
        if self.startup_timer is not None:
            self.startup_timer.cancel()
    
    def scan_callback(self):
        """Execute continuous oscillatory motion of joint_a1 - covers full 360°"""
        if self.current_joints is None:
            return
        
        # Increment/decrement angle based on direction
        angle_increment = (self.angular_velocity / self.update_rate) * self.scan_direction
        self.target_angle += angle_increment
        
        # Inverti direzione quando raggiungi i limiti
        if self.target_angle >= self.max_angle:
            self.target_angle = self.max_angle
            self.scan_direction = -1  # Inverti direzione
            self.get_logger().info('[SCAN] Reached max angle (+π) - reversing direction')
        elif self.target_angle <= self.min_angle:
            self.target_angle = self.min_angle
            self.scan_direction = 1  # Inverti direzione
            self.get_logger().info('[SCAN] Reached min angle (-π) - reversing direction')
        
        # Create trajectory message
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [
            'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4',
            'joint_a5', 'joint_a6', 'joint_a7'
        ]
        
        # Single waypoint
        point = JointTrajectoryPoint()
        point.positions = [
            self.target_angle,
            self.current_joints[1],
            self.current_joints[2],
            self.current_joints[3],
            self.current_joints[4],
            self.current_joints[5],
            self.current_joints[6]
        ]
        
        point.velocities = [self.angular_velocity * self.scan_direction, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=0, nanosec=int(2e9 / self.update_rate))
        
        traj.points.append(point)
        self.trajectory_pub.publish(traj)
        
        # Log più frequente per debug
        if int(self.target_angle * 10) % 10 == 0:  # Ogni ~0.1 rad
            self.get_logger().info(
                f'Rotating: target={self.target_angle:.3f} rad ({math.degrees(self.target_angle):.1f}°), '
                f'actual={self.current_joints[0]:.3f} rad ({math.degrees(self.current_joints[0]):.1f}°)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = IiwaScanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
