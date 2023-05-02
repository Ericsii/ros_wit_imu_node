import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Port for wit901 imu')
    declare_buadrate = DeclareLaunchArgument(
        'buadrate', default_value='115200',
        description='Buadrate for wit901 imu')
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='imu_link',
        description='Frame id for wit901 imu')
    declare_gravity = DeclareLaunchArgument(
        'gravity', default_value='9.80665',
        description='Gravity for wit901 imu')

    # Node
    node = Node(
        package='wit_imu_driver',
        executable='wit_imu_node',
        name='wit_imu_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('buadrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'gravity': LaunchConfiguration('gravity')
        }],
        namespace='imu'
    )

    ld = LaunchDescription()
    ld.add_action(declare_port)
    ld.add_action(declare_buadrate)
    ld.add_action(declare_frame_id)
    ld.add_action(declare_gravity)
    ld.add_action(node)

    return ld
