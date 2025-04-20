from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """Generate launch description for drone network."""
    
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim'],
            output='screen'
        ),

        # Launch MAVROS nodes for each drone
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone1',
            parameters=[{
                'fcu_url': 'udp://:14540@localhost:14557',
                'system_id': 1,
                'component_id': 1,
                'target_system_id': 1,
            }],
            output='screen'
        ),

        # Launch our drone network node
        Node(
            package='skyjackers_drone_network',
            executable='drone_network',
            name='drone_network',
            output='screen'
        )
    ])
