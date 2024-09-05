from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',   # Adjust this if your RPLIDAR is connected to a different port
                'serial_baudrate': 115200,        # A1's baudrate
                'frame_id': 'laser_1',        # Frame id for the lidar
                'inverted': False,                # Whether to invert the scan data
                'angle_compensate': True
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_1']
        ),
    ])
