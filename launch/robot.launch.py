from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for Ultra Magnus robot controller
    """
    
    # Declare launch arguments
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='1.0',
        description='Maximum robot speed in m/s'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.5',
        description='Minimum safe distance to obstacles in meters'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100',
        description='Control loop frequency in Hz'
    )
    
    # Robot controller node
    controller_node = Node(
        package='ultra_magnus',
        executable='robot_main',
        name='ultra_magnus_controller',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'safety_distance': LaunchConfiguration('safety_distance'),
            'control_frequency': LaunchConfiguration('control_frequency'),
        }],
        remappings=[
            ('/scan', '/laser/scan'),
            ('/odom', '/odometry/filtered'),
            ('/cmd_vel', '/mobile_base/commands/velocity'),
        ]
    )
    
    return LaunchDescription([
        max_speed_arg,
        safety_distance_arg,
        control_frequency_arg,
        controller_node,
    ])
