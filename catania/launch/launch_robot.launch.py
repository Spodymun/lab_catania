from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    catania_share = FindPackageShare('catania')
    velodyne_pointcloud_share = FindPackageShare('velodyne_pointcloud')

    xacro_file = PathJoinSubstitution([
        catania_share,
        'description',
        'robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    fake_drive_node = Node(
        package='catania',
        executable='fake_drive.py',
        output='screen',
        parameters=[{
            'wheel_radius': 0.035,
            'wheel_separation': 0.23,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'publish_rate': 30.0,
        }]
    )

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='screen'
    )

    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                velodyne_pointcloud_share,
                'launch',
                'velodyne_transform_node-VLP16-launch.py'
            ])
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([catania_share, 'rviz', 'basic.rviz'])],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        fake_drive_node,
        velodyne_driver_node,
        velodyne_transform_launch,
        rviz_node,
    ])