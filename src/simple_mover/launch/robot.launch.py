from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments (with sensible defaults)
    args = [
        DeclareLaunchArgument('tx', default_value='-0.40349150880660745',
                              description='Pos X'),
        DeclareLaunchArgument('ty', default_value='0.21355450702058534',
                              description='Pos Y'),
        DeclareLaunchArgument('tz', default_value='0.35585479647433027',
                              description='Pos Z'),
        DeclareLaunchArgument('rx', default_value='-0.9239217238827235',
                              description='Rotation X'),
        DeclareLaunchArgument('ry', default_value='0.3795637577378466',
                              description='Rotation Y'),
        DeclareLaunchArgument('rz', default_value='0.0017038598898846053',
                              description='Rotation Z'),
        DeclareLaunchArgument('rw', default_value='0.04792805870236521',
                              description='Rotation W'),
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


#ros2 launch your_pkg hello_moveit.launch.py \
#  tx:=-0.4 ty:=0.03 tz:=0.06 \
#  rx:=-0.93 ry:=0.36 rz:=-0.008 rw:=0.0356


