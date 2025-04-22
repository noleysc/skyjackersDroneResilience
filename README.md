# Skyjackers Encrypted Drone Network

A ROS 2 implementation of a multi-drone network with encrypted communication.

## features
- 5 drones in synchronized flight
- Encrypted communication between drones
- Circular flight paths with varying heights
- Position, velocity, and heading tracking
- Real-time monitoring and logs

## flight path
- Each drone follows a circular path
- Radius = 2.0 * drone_id
- Height offset = drone_id * 1.0
- Updates every 3 seconds
- Oscillating height pattern

## notes
- The drone network uses a shared encryption key
- Each drone follows a unique circular path to avoid collisions
- Update frequency is set to 3 seconds for clear monitoring
- Position data includes x, y, z coordinates and velocity (including direction) vectors
- Console displays real-time encrypted and decrypted messages

## usage
1. Build the workspace:
```bash
cd ~/drone_network_ws
colcon build
source install/setup.bash
ros2 launch skyjackers_drone_network drone_network.launch.py
