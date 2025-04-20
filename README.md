# Skyjackers Encrypted Drone Network

A ROS 2 implementation of a multi-drone network with encrypted communication.

## features
- 5 drones in synchronized flight
- Encrypted communication between drones
- Circular flight paths with varying heights
- Position, velocity, and heading tracking
- Real-time monitoring

## flight path
- Each drone follows a circular path
- Radius = 2.0 * drone_id
- Height offset = drone_id * 1.0
- Updates every 3 seconds
- Oscillating height pattern

## usage
1. Build the workspace:
```bash
colcon build
source install/setup.bash

## notes
- Written on Ubuntu 24.4.2 (Noble) using VirtualBox (latest) VM.
