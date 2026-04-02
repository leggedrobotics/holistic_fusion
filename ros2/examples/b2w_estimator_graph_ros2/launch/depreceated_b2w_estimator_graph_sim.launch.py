from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package name
    pkg_name = 'b2w_estimator_graph_ros2'
    
    # Get package share directory
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic_name = LaunchConfiguration('imu_topic_name')
    lidar_odometry_topic_name = LaunchConfiguration('lidar_odometry_topic_name')
    vio_odometry_topic_name = LaunchConfiguration('vio_odometry_topic_name')
    logging_dir_location = LaunchConfiguration('logging_dir_location')

    # Default config file paths
    core_graph_config_param_file = os.path.join(pkg_dir, 'config', 'core', 'core_graph_config.yaml')
    core_graph_param_file = os.path.join(pkg_dir, 'config', 'core', 'core_graph_params.yaml')
    core_extrinsic_param_file = os.path.join(pkg_dir, 'config', 'core', 'core_extrinsic_params.yaml')
    b2w_graph_param_file = os.path.join(pkg_dir, 'config', 'b2w_specific', 'b2w_graph_params.yaml')
    b2w_extrinsic_param_file = os.path.join(pkg_dir, 'config', 'b2w_specific', 'b2w_extrinsic_params.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'imu_topic_name',
            default_value='/imu/data_raw',
            description='IMU topic name'
        ),
        DeclareLaunchArgument(
            'lidar_odometry_topic_name',
            default_value='/open3d/scan2map_odometry',
            description='Lidar odometry topic name'
        ),
        DeclareLaunchArgument(
            'vio_odometry_topic_name',
            default_value='/tracking_camera/odom/sample',
            description='VIO odometry topic name'
        ),
        DeclareLaunchArgument(
            'logging_dir_location',
            default_value=os.path.join(pkg_dir, 'logging'),
            description='Logging directory location'
        ),

        # Node
        Node(
            package='b2w_estimator_graph_ros2',
            executable='b2w_estimator_graph_ros2_node',
            name='b2w_estimator_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'launch/optimizationResultLoggingPath': logging_dir_location},
                core_graph_config_param_file,
                core_graph_param_file,
                core_extrinsic_param_file,
                b2w_graph_param_file,
                b2w_extrinsic_param_file
            ],
            remappings=[
                ('/imu_topic', imu_topic_name),
                ('/lidar_odometry_topic', lidar_odometry_topic_name),
                ('/vio_odometry_topic', vio_odometry_topic_name)
            ]
        )
    ]) 
