from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package name
    pkg_name = "smb_estimator_graph_ros2"

    # Get package share directory
    pkg_dir = get_package_share_directory(pkg_name)

    # Sim Time
    use_sim_time_str = launch.substitutions.LaunchConfiguration(
        "use_sim_time", default="true"
    ).perform(launch.LaunchContext())
    use_sim_time = use_sim_time_str.lower() == "true"  # Convert to Python bool
    print(f"Use Sim Time: {use_sim_time}")

    # Declare launch arguments
    imu_topic_name = LaunchConfiguration("imu_topic_name")
    lidar_odometry_topic_name = LaunchConfiguration("lidar_odometry_topic_name")
    core_graph_param_file = LaunchConfiguration("core_graph_param_file")
    smb_graph_param_file = LaunchConfiguration("smb_graph_param_file")

    # Default config file paths
    default_core_graph_param_file = os.path.join(
        pkg_dir, "config", "core", "core_graph_params.yaml"
    )
    default_smb_graph_param_file = os.path.join(
        pkg_dir, "config", "smb_specific", "smb_graph_params.yaml"
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            # A. Sim time
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time_str.lower(),
                description="Use simulated time",
            ),
            # B. Topics
            DeclareLaunchArgument(
                "imu_topic_name", default_value="/imu/data_raw", description="IMU topic name"
            ),
            DeclareLaunchArgument(
                "lidar_odometry_topic_name",
                default_value="/open3d/scan2map_odometry",
                description="Lidar odometry topic name",
            ),
            # C. Parameter files
            DeclareLaunchArgument(
                "core_graph_param_file",
                default_value=default_core_graph_param_file,
                description="Core graph parameter file",
            ),
            DeclareLaunchArgument(
                "smb_graph_param_file",
                default_value=default_smb_graph_param_file,
                description="SMB graph parameter file",
            ),
            # D. Set the /use_sim_time parameter for all nodes
            launch.actions.SetLaunchConfiguration(
                "use_sim_time", launch.substitutions.LaunchConfiguration("use_sim_time")
            ),
            # E. Include the main launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_dir, "launch", "smb_estimator_graph.launch.py")]
                ),
                launch_arguments={
                    "use_sim_time": launch.substitutions.LaunchConfiguration(
                        "use_sim_time"
                    ),
                    "imu_topic_name": imu_topic_name,
                    "lidar_odometry_topic_name": lidar_odometry_topic_name,
                    "core_graph_param_file": core_graph_param_file,
                    "smb_graph_param_file": smb_graph_param_file,
                }.items(),
            ),
            # F. RVIZ Node
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(pkg_dir, "rviz", "rviz_config.rviz")],
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
        ]
    )
