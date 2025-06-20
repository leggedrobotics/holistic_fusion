from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package name
    pkg_name = "smb_estimator_graph_ros2"

    # Get package share directory
    pkg_dir = get_package_share_directory(pkg_name)

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    imu_topic_name = LaunchConfiguration("imu_topic_name")
    lidar_odometry_topic_name = LaunchConfiguration("lidar_odometry_topic_name")
    wheel_odometry_topic_name = LaunchConfiguration("wheel_odometry_topic_name")
    wheel_velocities_topic_name = LaunchConfiguration("wheel_velocities_topic_name")
    vio_odometry_topic_name = LaunchConfiguration("vio_odometry_topic_name")
    logging_dir_location = LaunchConfiguration("logging_dir_location")

    # Default config file paths
    core_graph_config_param_file = os.path.join(
        pkg_dir, "config", "core", "core_graph_config.yaml"
    )
    core_graph_param_file = os.path.join(
        pkg_dir, "config", "core", "core_graph_params.yaml"
    )
    core_extrinsic_param_file = os.path.join(
        pkg_dir, "config", "core", "core_extrinsic_params.yaml"
    )
    smb_graph_param_file = os.path.join(
        pkg_dir, "config", "smb_specific", "smb_graph_params.yaml"
    )
    smb_extrinsic_param_file = os.path.join(
        pkg_dir, "config", "smb_specific", "smb_extrinsic_params.yaml"
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time", default_value=use_sim_time, description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "imu_topic_name", default_value="/imu/data_raw", description="IMU topic name"
            ),
            DeclareLaunchArgument(
                "lidar_odometry_topic_name",
                default_value="/open3d/scan2map_odometry",
                description="Lidar odometry topic name",
            ),
            DeclareLaunchArgument(
                "wheel_odometry_topic_name",
                default_value="/control/smb_diff_drive/odom",
                description="Wheel odometry topic name",
            ),
            DeclareLaunchArgument(
                "wheel_velocities_topic_name",
                default_value="/control/smb_lowlevel_controller/wheelSpeeds",
                description="Wheel velocities topic name",
            ),
            DeclareLaunchArgument(
                "vio_odometry_topic_name",
                default_value="/tracking_camera/odom/sample",
                description="VIO odometry topic name",
            ),
            DeclareLaunchArgument(
                "logging_dir_location",
                default_value=os.path.join(pkg_dir, "logging"),
                description="Logging directory location",
            ),
            # Set the /use_sim_time parameter for all nodes
            launch.actions.SetLaunchConfiguration("use_sim_time", launch.substitutions.LaunchConfiguration("use_sim_time")),
            # Main Node
            Node(
                package="smb_estimator_graph_ros2",
                executable="smb_estimator_graph_ros2_node",
                name="smb_estimator_node",
                output="screen",
                # prefix='gdb -ex run --args',
                # prefix='xterm -e gdb -ex run --args',
                # emulate_tty=True,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"launch/optimizationResultLoggingPath": logging_dir_location},
                    core_graph_config_param_file,
                    core_graph_param_file,
                    core_extrinsic_param_file,
                    smb_graph_param_file,
                    smb_extrinsic_param_file,
                ],
                remappings=[
                    ("/imu_topic", imu_topic_name),
                    ("/lidar_odometry_topic", lidar_odometry_topic_name),
                    ("/wheel_odometry_topic", wheel_odometry_topic_name),
                    ("/wheel_velocities_topic", wheel_velocities_topic_name),
                    ("/vio_odometry_topic", vio_odometry_topic_name),
                ],
            ),
        ]
    )
