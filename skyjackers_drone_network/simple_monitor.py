#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class SimpleMonitor(Node):
    def __init__(self):
        super().__init__('simple_monitor')
        
        # Create subscribers for each drone
        for i in range(1, 6):
            self.create_subscription(
                PoseStamped,
                f'/drone{i}/position',
                lambda msg, drone_id=i: self.position_callback(msg, drone_id),
                10
            )
        
        self.get_logger().info('Simple Monitor Started')

    def position_callback(self, msg, drone_id):
        """Handle position updates"""
        self.get_logger().info(
            f'Drone {drone_id} position: '
            f'x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, '
            f'z={msg.pose.position.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    monitor = SimpleMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
