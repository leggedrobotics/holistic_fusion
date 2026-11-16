from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = "b2w_estimator_graph_ros2"
    pkg_dir = get_package_share_directory(pkg_name)
    rviz_dbus_address = os.environ.get("DBUS_SESSION_BUS_ADDRESS") or "unix:path=/dev/null"

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gnss_unary = LaunchConfiguration("use_gnss_unary")
    imu_topic_name = LaunchConfiguration("imu_topic_name")
    lidar_odometry_topic_name = LaunchConfiguration("lidar_odometry_topic_name")
    lio_degeneracy_topic_name = LaunchConfiguration("lio_degeneracy_topic_name")
    between_lidar_odometry_topic_name = LaunchConfiguration("between_lidar_odometry_topic_name")
    gnss_topic_name = LaunchConfiguration("gnss_topic_name")
    vio_odometry_topic_name = LaunchConfiguration("vio_odometry_topic_name")
    vio_odometry_between_topic_name = LaunchConfiguration("vio_odometry_between_topic_name")
    logging_dir_location = LaunchConfiguration("logging_dir_location")
    use_rviz = LaunchConfiguration("use_rviz")
    extra_params_file = LaunchConfiguration("extra_params_file")

    # Static parameter files
    core_graph_config_param_file = os.path.join(pkg_dir, "config", "core", "core_graph_config.yaml")
    core_extrinsic_param_file = os.path.join(pkg_dir, "config", "core", "core_extrinsic_params.yaml")
    trajectory_alignment_param_file = os.path.join(pkg_dir, "config", "b2w_specific", "b2w_traj_align_params.yaml")
    b2w_extrinsic_param_file = os.path.join(pkg_dir, "config", "b2w_specific", "b2w_extrinsic_params.yaml")
    b2w_gnss_param_file = os.path.join(pkg_dir, "config", "b2w_specific", "b2w_gnss_params.yaml")

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("use_gnss_unary", default_value="true", description="Enable GNSS unary factor"),
        DeclareLaunchArgument("imu_topic_name", default_value="/imu_sensor_broadcaster/imu", description="IMU topic name"),
        DeclareLaunchArgument("lidar_odometry_topic_name", default_value="/dlio/odom_node/map_pose", description="Lidar odometry topic name"),
        DeclareLaunchArgument("lio_degeneracy_topic_name", default_value="/dlio/odom_node/degenerate", description="Lidar degeneracy topic name"),
        DeclareLaunchArgument("between_lidar_odometry_topic_name", default_value="/dlio/odom_node/odom", description="Between lidar odometry topic name (nav_msgs/Odometry)"),
        DeclareLaunchArgument("vio_odometry_topic_name", default_value="/zed/zed_node/pose_with_covariance", description="VIO odometry topic name"),
        DeclareLaunchArgument("vio_odometry_between_topic_name", default_value="/zed/zed_node/odom", description="Between VIO odometry topic name"),
        DeclareLaunchArgument("gnss_topic_name", default_value="/navsatfix", description="GNSS topic name"),
        DeclareLaunchArgument("joint_states_topic_name", default_value="/joint_states", description="Joint states (wheel odometry)"),
        DeclareLaunchArgument("logging_dir_location", default_value=os.path.join(pkg_dir, "logging"), description="Logging directory location"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz alongside the estimator"),
        DeclareLaunchArgument("extra_params_file", default_value=os.path.join(pkg_dir, "config", "overlays", "none.yaml"),
                              description="Parameter overlay loaded last; overrides any parameter of the estimator node"),

        # Main Node without GNSS
        Node(
            package="b2w_estimator_graph_ros2",
            executable="b2w_estimator_graph_ros2_node",
            name="b2w_estimator_node",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"launch/optimizationResultLoggingPath": logging_dir_location},
                core_graph_config_param_file,
                trajectory_alignment_param_file,
                b2w_gnss_param_file,
                os.path.join(pkg_dir, "config", "core", "core_graph_params_gnss.yaml"),
                core_extrinsic_param_file,
                os.path.join(pkg_dir, "config", "b2w_specific", "b2w_graph_params_gnss.yaml"),
                os.path.join(pkg_dir, "config", "b2w_specific", "hf_b2w_description.yaml"),
                b2w_extrinsic_param_file,
                extra_params_file,
            ],
            remappings=[
                ("/imu_topic", imu_topic_name),
                ("/lidar_odometry_topic", lidar_odometry_topic_name),
                ("/lio_degeneracy_topic", lio_degeneracy_topic_name),
                ("/between_lidar_odometry_topic", between_lidar_odometry_topic_name),
                ("/vio_odometry_topic", vio_odometry_topic_name),
                ("/vio_odometry_between_topic", vio_odometry_between_topic_name),
                ("/gnss_topic", gnss_topic_name),
                ("/joint_states_topic", LaunchConfiguration("joint_states_topic_name")),
            ],
            # condition=UnlessCondition(use_gnss_unary),
        ),

            # F. RVIZ Node
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", os.path.join(pkg_dir, "rviz", "rviz_config.rviz"),
                "--ros-args", "--log-level", "rviz2:=warn",
                "--ros-args", "--log-level", "tf2_ros:=error",
                "--ros-args", "--log-level", "message_filters:=error",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            # Prevent libdbus from auto-launching a private session bus in
            # containers that do not expose the host desktop session bus.
            # That private connection aborts RViz during shutdown/cleanup.
            additional_env={"DBUS_SESSION_BUS_ADDRESS": rviz_dbus_address},
            output="log",
            condition=IfCondition(use_rviz),
        ),
    ])
