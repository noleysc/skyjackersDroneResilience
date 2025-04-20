#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from std_msgs.msg import String
import json
import threading
import base64
import math
import time

class SimpleDroneController(Node):
    def __init__(self, drone_id):
        super().__init__(f'drone_{drone_id}')
        
        self.drone_id = drone_id
        self.drone_positions = {}
        
        # Simple encryption key
        self.encryption_key = b'drone_network_key123'
        
        # Flight path parameters
        self.time_counter = 0
        self.radius = 2.0 * drone_id
        self.height_offset = drone_id * 1.0
        
        # Publishers
        self.position_pub = self.create_publisher(
            PoseStamped,
            f'/drone{drone_id}/position',  # Topic name for position
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/drone_network/status',
            10
        )
        
        # Initialize position
        self.current_position = PoseStamped()
        self.current_position.header.frame_id = 'map'  # Add frame_id
        self.current_position.pose.position.x = float(drone_id)
        self.current_position.pose.position.y = 0.0
        self.current_position.pose.position.z = self.height_offset
        
        # Create timer for updates (3 seconds)
        self.create_timer(3.0, self.timer_callback)
        
        self.get_logger().info(f'Drone {drone_id} initialized')
        self.get_logger().info(f'Publishing position on: /drone{drone_id}/position')
        self.test_encryption()

    def timer_callback(self):
        """Regular updates"""
        # Calculate new position
        flight_data = self.calculate_position()
        
        # Update and publish position
        self.current_position.header.stamp = self.get_clock().now().to_msg()
        self.current_position.pose.position.x = flight_data['position']['x']
        self.current_position.pose.position.y = flight_data['position']['y']
        self.current_position.pose.position.z = flight_data['position']['z']
        
        # Publish position
        self.position_pub.publish(self.current_position)
        self.get_logger().info(f'Published position for drone{self.drone_id}')
    def calculate_position(self):
        """Calculate new position based on time"""
        # Circular motion in XY plane with varying height
        self.time_counter += 0.1
        
        # Calculate new position
        x = self.radius * math.cos(self.time_counter * 0.5)
        y = self.radius * math.sin(self.time_counter * 0.5)
        z = self.height_offset + math.sin(self.time_counter) * 0.5  # Oscillating height
        
        # Calculate velocities
        vx = -self.radius * 0.5 * math.sin(self.time_counter * 0.5)
        vy = self.radius * 0.5 * math.cos(self.time_counter * 0.5)
        vz = math.cos(self.time_counter) * 0.5
        
        # Calculate heading (yaw) based on velocity direction
        heading = math.atan2(vy, vx)
        
        return {
            'position': {'x': x, 'y': y, 'z': z},
            'velocity': {'vx': vx, 'vy': vy, 'vz': vz},
            'heading': heading
        }

    def encrypt(self, message):
        """Simple XOR encryption"""
        if isinstance(message, str):
            message = message.encode()
        encrypted = bytearray()
        for i in range(len(message)):
            encrypted.append(message[i] ^ self.encryption_key[i % len(self.encryption_key)])
        return base64.b64encode(encrypted).decode()

    def decrypt(self, encrypted_message):
        """Simple XOR decryption"""
        try:
            encrypted = base64.b64decode(encrypted_message)
            decrypted = bytearray()
            for i in range(len(encrypted)):
                decrypted.append(encrypted[i] ^ self.encryption_key[i % len(self.encryption_key)])
            return decrypted.decode()
        except Exception as e:
            self.get_logger().error(f'Decryption error: {str(e)}')
            return None

    def test_encryption(self):
        """Test encryption/decryption"""
        test_data = {
            'drone_id': self.drone_id,
            'position': {'x': 1.0, 'y': 2.0, 'z': 3.0}
        }
        test_message = json.dumps(test_data)
        self.get_logger().info(f'\nENCRYPTION TEST:')
        self.get_logger().info(f'Original: {test_message}')
        encrypted = self.encrypt(test_message)
        self.get_logger().info(f'Encrypted: {encrypted}')
        decrypted = self.decrypt(encrypted)
        self.get_logger().info(f'Decrypted: {decrypted}')
        if test_message == decrypted:
            self.get_logger().info('Encryption test PASSED!')
        else:
            self.get_logger().error('Encryption test FAILED!')

    def timer_callback(self):
        """Regular updates"""
        # Calculate new position
        flight_data = self.calculate_position()
        
        # Update position
        self.current_position.pose.position.x = flight_data['position']['x']
        self.current_position.pose.position.y = flight_data['position']['y']
        self.current_position.pose.position.z = flight_data['position']['z']
        
        # Create status message with extended information
        status_data = {
            'drone_id': self.drone_id,
            'timestamp': time.time(),
            'position': flight_data['position'],
            'velocity': flight_data['velocity'],
            'heading': flight_data['heading'],
            'radius': self.radius,
            'height_offset': self.height_offset
        }
        
        # Convert to JSON string
        json_str = json.dumps(status_data)
        
        # Encrypt the JSON string
        encrypted_data = self.encrypt(json_str)
        
        # Publish encrypted status
        status_msg = String()
        status_msg.data = encrypted_data
        self.status_pub.publish(status_msg)
        
        # Debug output
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'Drone {self.drone_id} Status Update:')
        self.get_logger().info(f'Position: (x={flight_data["position"]["x"]:.2f}, '
                              f'y={flight_data["position"]["y"]:.2f}, '
                              f'z={flight_data["position"]["z"]:.2f})')
        self.get_logger().info(f'Velocity: (vx={flight_data["velocity"]["vx"]:.2f}, '
                              f'vy={flight_data["velocity"]["vy"]:.2f}, '
                              f'vz={flight_data["velocity"]["vz"]:.2f})')
        self.get_logger().info(f'Heading: {math.degrees(flight_data["heading"]):.2f} degrees')
        self.get_logger().info(f'Encrypted Length: {len(encrypted_data)} bytes')

    def status_callback(self, msg):
        """Handle encrypted status messages"""
        try:
            # Decrypt
            decrypted_data = self.decrypt(msg.data)
            
            if decrypted_data:
                status_data = json.loads(decrypted_data)
                if status_data['drone_id'] != self.drone_id:  # Ignore own messages
                    self.get_logger().info(f'\nReceived from Drone {status_data["drone_id"]}:')
                    self.get_logger().info(
                        f'Position: (x={status_data["position"]["x"]:.2f}, '
                        f'y={status_data["position"]["y"]:.2f}, '
                        f'z={status_data["position"]["z"]:.2f})'
                    )
                    self.get_logger().info(
                        f'Velocity: (vx={status_data["velocity"]["vx"]:.2f}, '
                        f'vy={status_data["velocity"]["vy"]:.2f}, '
                        f'vz={status_data["velocity"]["vz"]:.2f})'
                    )
                    self.get_logger().info(
                        f'Heading: {math.degrees(status_data["heading"]):.2f} degrees'
                    )
                    self.drone_positions[status_data['drone_id']] = status_data['position']
        except Exception as e:
            self.get_logger().error(f'Status processing error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create network of 5 drones
    drones = []
    try:
        for i in range(5):
            drone = SimpleDroneController(i+1)
            drones.append(drone)
        
        # Spin all drones in separate threads
        executor = rclpy.executors.MultiThreadedExecutor()
        for drone in drones:
            executor.add_node(drone)
        
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        rclpy.spin(drones[0])  # Keep main thread alive
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        for drone in drones:
            drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
