from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments (with sensible defaults)
    args = [
        DeclareLaunchArgument('tx', default_value='0.0',
                              description='Pose X (meters)'),
        DeclareLaunchArgument('ty', default_value='0.0',
                              description='Pose Y (meters)'),
        DeclareLaunchArgument('tz', default_value='0.0',
                              description='Pose Z (meters)'),
        DeclareLaunchArgument('rx', default_value='0.0',
                              description='Orientation X'),
        DeclareLaunchArgument('ry', default_value='0.0',
                              description='Orientation Y'),
        DeclareLaunchArgument('rz', default_value='0.0',
                              description='Orientation Z'),
        DeclareLaunchArgument('rw', default_value='1.0',
                              description='Orientation W'),
    ]

    # Node that spins your MoveIt! example
    hello_moveit_node = Node(
        package='simple_mover',         
        executable='mover_auto',           
        name='hello_moveit2',
        output='screen',
        parameters=[
            {'tx': LaunchConfiguration('tx')},
            {'ty': LaunchConfiguration('ty')},
            {'tz': LaunchConfiguration('tz')},
            {'rx': LaunchConfiguration('rx')},
            {'ry': LaunchConfiguration('ry')},
            {'rz': LaunchConfiguration('rz')},
            {'rw': LaunchConfiguration('rw')},
        ],
    )

    return LaunchDescription(args + [hello_moveit_node])
