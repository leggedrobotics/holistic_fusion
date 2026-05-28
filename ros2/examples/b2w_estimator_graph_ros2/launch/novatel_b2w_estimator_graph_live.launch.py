from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = "b2w_estimator_graph_ros2"
    pkg_dir = get_package_share_directory(pkg_name)

    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_topic_name = LaunchConfiguration("imu_topic_name")
    lidar_odometry_topic_name = LaunchConfiguration("lidar_odometry_topic_name")
    between_lidar_odometry_topic_name = LaunchConfiguration("between_lidar_odometry_topic_name")
    gnss_topic_name = LaunchConfiguration("gnss_topic_name")
    vio_odometry_topic_name = LaunchConfiguration("vio_odometry_topic_name")
    vio_odometry_between_topic_name = LaunchConfiguration("vio_odometry_between_topic_name")
    logging_dir_location = LaunchConfiguration("logging_dir_location")

    heading2_topic = LaunchConfiguration("heading2_topic")
    initial_yaw_topic = LaunchConfiguration("initial_yaw_topic")
    sensor_frame_id = LaunchConfiguration("sensor_frame_id")
    heading_yaw_offset_deg = LaunchConfiguration("heading_yaw_offset_deg")
    use_initial_heading = LaunchConfiguration("use_initial_heading")
    initial_heading_max_age_sec = LaunchConfiguration("initial_heading_max_age_sec")
    initial_heading_wait_timeout_sec = LaunchConfiguration("initial_heading_wait_timeout_sec")

    core_graph_config_param_file = os.path.join(pkg_dir, "config", "core", "core_graph_config.yaml")
    core_extrinsic_param_file = os.path.join(pkg_dir, "config", "core", "core_extrinsic_params.yaml")
    trajectory_alignment_param_file = os.path.join(pkg_dir, "config", "b2w_specific", "b2w_traj_align_params.yaml")
    b2w_extrinsic_param_file = os.path.join(pkg_dir, "config", "b2w_specific", "b2w_extrinsic_params.yaml")
    b2w_gnss_param_file = os.path.join(pkg_dir, "config", "b2w_specific", "b2w_gnss_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("imu_topic_name", default_value="/gt_box/livox/imu_si_compliant", description="IMU topic name"),
        DeclareLaunchArgument("lidar_odometry_topic_name", default_value="/dlio/odom_node/map_pose",
                              description="Lidar odometry topic name"),
        DeclareLaunchArgument("between_lidar_odometry_topic_name", default_value="/dlio2/odom_node/odom22",
                              description="Between lidar odometry topic name"),
        DeclareLaunchArgument("vio_odometry_topic_name", default_value="/zed/zed_node/pose_with_covariance",
                              description="VIO odometry topic name"),
        DeclareLaunchArgument("vio_odometry_between_topic_name", default_value="/zed/zed_node/odom",
                              description="Between VIO odometry topic name"),
        DeclareLaunchArgument("gnss_topic_name", default_value="/gnss/novatel/fix",
                              description="Estimator live GNSS NavSatFix topic"),
        DeclareLaunchArgument("logging_dir_location", default_value=os.path.join(pkg_dir, "logging"),
                              description="Logging directory location"),

        DeclareLaunchArgument("heading2_topic", default_value="/gt_box/cpt7/heading2",
                              description="Bridged NovAtel HEADING2 topic"),
        DeclareLaunchArgument("initial_yaw_topic", default_value="/gnss/initial_yaw",
                              description="Converted initial yaw output topic"),
        DeclareLaunchArgument("sensor_frame_id", default_value="cpt7_heading2_base_to_rover",
                              description="Header frame_id for the converted HEADING2 base-to-rover yaw message"),
        DeclareLaunchArgument("heading_yaw_offset_deg", default_value="0.0",
                              description="Yaw offset from NovAtel HEADING2 yaw to sensor_frame_id"),
        DeclareLaunchArgument("use_initial_heading", default_value="true",
                              description="Use converted HEADING2 as estimator initial yaw"),
        DeclareLaunchArgument("initial_heading_max_age_sec", default_value="2.0",
                              description="Maximum age of initial yaw relative to GNSS time"),
        DeclareLaunchArgument("initial_heading_wait_timeout_sec", default_value="3.0",
                              description="Legacy wait parameter; heading-enabled startup waits for fresh yaw without alignment fallback"),

        Node(
            package="b2w_estimator_graph_ros2",
            executable="novatel_oem7_adapter_node",
            name="novatel_oem7_adapter",
            output="screen",
            condition=IfCondition(use_initial_heading),
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {"heading2_topic": heading2_topic},
                {"initial_yaw_topic": initial_yaw_topic},
                {"heading_yaw_offset_deg": ParameterValue(heading_yaw_offset_deg, value_type=float)},
                {"sensor_frame_id": sensor_frame_id},
            ],
        ),

        Node(
            package="b2w_estimator_graph_ros2",
            executable="b2w_estimator_graph_ros2_node",
            name="b2w_estimator_node",
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {"launch/optimizationResultLoggingPath": logging_dir_location},
                core_graph_config_param_file,
                trajectory_alignment_param_file,
                b2w_gnss_param_file,
                os.path.join(pkg_dir, "config", "core", "core_graph_params_gnss.yaml"),
                core_extrinsic_param_file,
                os.path.join(pkg_dir, "config", "b2w_specific", "b2w_graph_params_gnss.yaml"),
                b2w_extrinsic_param_file,
                {"gnss_params.useYawInitialGuessFromHeading": ParameterValue(use_initial_heading, value_type=bool)},
                {"gnss_params.initialHeadingMaxAgeSec": ParameterValue(initial_heading_max_age_sec, value_type=float)},
                {"gnss_params.initialHeadingWaitTimeoutSec": ParameterValue(initial_heading_wait_timeout_sec, value_type=float)},
            ],
            remappings=[
                ("/imu_topic", imu_topic_name),
                ("/lidar_odometry_topic", lidar_odometry_topic_name),
                ("/between_lidar_odometry_topic", between_lidar_odometry_topic_name),
                ("/vio_odometry_topic", vio_odometry_topic_name),
                ("/vio_odometry_between_topic", vio_odometry_between_topic_name),
                ("/gnss_topic", gnss_topic_name),
                ("/initial_yaw_topic", initial_yaw_topic),
            ],
        ),
    ])
