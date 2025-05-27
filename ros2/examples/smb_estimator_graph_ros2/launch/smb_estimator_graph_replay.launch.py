from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package name
    pkg_name = 'smb_estimator_graph_ros2'
    
    # Get package share directory
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic_name = LaunchConfiguration('imu_topic_name')
    lidar_odometry_topic_name = LaunchConfiguration('lidar_odometry_topic_name')
    core_graph_param_file = LaunchConfiguration('core_graph_param_file')
    smb_graph_param_file = LaunchConfiguration('smb_graph_param_file')

    # Default config file paths
    default_core_graph_param_file = os.path.join(pkg_dir, 'config', 'core', 'core_graph_params.yaml')
    default_smb_graph_param_file = os.path.join(pkg_dir, 'config', 'smb_specific', 'smb_graph_params.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'imu_topic_name',
            default_value='/imu',
            description='IMU topic name'
        ),
        DeclareLaunchArgument(
            'lidar_odometry_topic_name',
            default_value='/mapping/scan2map_odometry',
            description='Lidar odometry topic name'
        ),
        DeclareLaunchArgument(
            'core_graph_param_file',
            default_value=default_core_graph_param_file,
            description='Core graph parameter file'
        ),
        DeclareLaunchArgument(
            'smb_graph_param_file',
            default_value=default_smb_graph_param_file,
            description='SMB graph parameter file'
        ),

        # Include the main launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'smb_estimator_graph.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'imu_topic_name': imu_topic_name,
                'lidar_odometry_topic_name': lidar_odometry_topic_name,
                'core_graph_param_file': core_graph_param_file,
                'smb_graph_param_file': smb_graph_param_file
            }.items()
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'lidar_estimation.rviz')],
            output='screen'
        )
    ])