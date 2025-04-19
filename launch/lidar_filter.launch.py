from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name_parameter = DeclareLaunchArgument(
        'robot_name',
        default_value='RobotName')

    name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        name_parameter,
        Node(
            package='sensing',
            executable='lidar_filter',
            namespace=name,
            name='front_lidar_filter',
            output='screen',
            parameters=[{
                    'lidar_topic': ['/', name, '/front_scan'],
                    'imu_topic': ['/', name, '/imu'],
                    'output_topic': ['/', name, '/filtered_front_scan'],
                    'base_frame': [name, '/base_footprint'],
                    'use_sim_time': True
            }]
        ),
        Node(
            package='sensing',
            executable='lidar_filter',
            namespace=name,
            name='back_lidar_filter',
            output='screen',
            parameters=[{
                    'lidar_topic': ['/', name, '/back_scan'],
                    'imu_topic': ['/', name, '/imu'],
                    'output_topic': ['/', name, '/filtered_back_scan'],
                    'base_frame': [name, '/base_footprint'],
                    'use_sim_time': True
            }]
        ),
        Node(
            package='sensing',
            executable='lidar_merger',
            namespace=name,
            name='lidar_merger',
            output='screen',
            parameters=[{
                    'lidar_topic1': ['/', name, '/filtered_front_scan'],
                    'lidar_topic2': ['/', name, '/filtered_back_scan'],
                    'output_topic': ['/', name, '/merged_scan'],
                    'base_frame': [name, '/base_footprint'],
                    'use_sim_time': True
            }]
        ),
        Node(
            package='sensing',
            executable='grid_filter',
            namespace=name,
            name='front_grid_filter',
            output='screen',
            parameters=[{
                    'input_topic': ['/', name, '/merged_scan'],
                    'output_topic': ['/', name, '/downsampled_scan'],
                    'base_frame': [name, '/base_footprint'],
                    'use_sim_time': True
            }]
        ),
    ])
