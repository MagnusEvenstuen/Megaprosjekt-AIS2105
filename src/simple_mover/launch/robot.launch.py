from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments (with sensible defaults)
    args = [
        DeclareLaunchArgument('tx', default_value='-0.36747354041927566',
                              description='Camera Pos X'),
        DeclareLaunchArgument('ty', default_value='0.3294852403337859',
                              description='Camera Pos Y'),
        DeclareLaunchArgument('tz', default_value='0.6',
                              description='Pos Z'),
        DeclareLaunchArgument('rx', default_value='-0.9216443017139829',
                              description='Rotation X'),
        DeclareLaunchArgument('ry', default_value='0.3868741666290205',
                              description='Rotation Y'),
        DeclareLaunchArgument('rz', default_value='-0.007769835299300791',
                              description='Rotation Z'),
        DeclareLaunchArgument('rw', default_value='0.028979129950146974',
                              description='Rotation W'),
        DeclareLaunchArgument('image_width', default_value='640',
                              description='Image Width'),
        DeclareLaunchArgument('image_height', default_value='480',
                              description='Image Height'),
        DeclareLaunchArgument('sleep_x', default_value='-0.367',
                              description='Sleep Position X'),
        DeclareLaunchArgument('sleep_y', default_value='0.329',
                              description='Sleep Position Y'),
        DeclareLaunchArgument('sleep_z', default_value='0.200',
                              description='Sleep Position Z'),
        DeclareLaunchArgument('home', default_value='false',
                              description='Move directly to sleep position'),
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
            {'sleep_x': LaunchConfiguration('sleep_x')},
            {'sleep_y': LaunchConfiguration('sleep_y')},
            {'sleep_z': LaunchConfiguration('sleep_z')},
            {'home': LaunchConfiguration('home')},
        ],
    )

    camera_detector_node = Node(
        package='camera_detector',
        executable='camera_detector',
        name='cube_detector',
        output='screen'
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_camera',
        parameters=[{
            'video_device': '/dev/video2',
            'frame_id': 'camera_frame',
            'image_width': 640,
            'image_height': 480,
        }],
        output='screen'
    )

    return LaunchDescription(args + [hello_moveit_node, camera_detector_node, camera_node])