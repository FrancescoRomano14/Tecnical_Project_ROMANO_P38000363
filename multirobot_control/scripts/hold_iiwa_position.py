#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time


class IiwaPositionSetter(Node):
    def __init__(self):
        super().__init__('iiwa_position_setter')
        
        # Aspetta che Gazebo sia pronto
        time.sleep(3.0)
        
        # Posizioni desiderate (posizione a L verso fra2mo)
        joint_positions = {
            'joint_a1': -1.57,  # -90° verso fra2mo
            'joint_a2': -0.5,   # solleva spalla
            'joint_a3': 0.0,    # neutro
            'joint_a4': -1.57,  # -90° gomito (forma L)
            'joint_a5': 0.0,    # neutro
            'joint_a6': 0.0,    # camera orizzontale
            'joint_a7': 0.0     # neutro
        }
        
        self.get_logger().info('Setting iiwa joints to L position...')
        
        # Imposta ogni joint usando il servizio Ignition
        for joint_name, position in joint_positions.items():
            cmd = [
                'ign', 'service',
                '-s', '/world/empty/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "iiwa", id: 0'
            ]
            
            # Comando alternativo per impostare joint
            joint_cmd = [
                'gz', 'topic',
                '-t', f'/model/iiwa/joint/{joint_name}/cmd_pos',
                '-m', 'gz.msgs.Double',
                '-p', f'data: {position}'
            ]
            
            try:
                subprocess.run(joint_cmd, check=False)
                self.get_logger().info(f'Set {joint_name} = {position}')
            except Exception as e:
                self.get_logger().error(f'Failed to set {joint_name}: {e}')
        
        self.get_logger().info('Iiwa positioned!')


def main(args=None):
    rclpy.init(args=args)
    node = IiwaPositionSetter()
    
    # Mantieni il nodo vivo
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
