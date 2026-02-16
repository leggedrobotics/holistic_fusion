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

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gnss_unary = LaunchConfiguration("use_gnss_unary")
    imu_topic_name = LaunchConfiguration("imu_topic_name")
    lidar_odometry_topic_name = LaunchConfiguration("lidar_odometry_topic_name")
    between_lidar_odometry_topic_name = LaunchConfiguration("between_lidar_odometry_topic_name")
    gnss_topic_name = LaunchConfiguration("gnss_topic_name")
    wheel_odometry_topic_name = LaunchConfiguration("wheel_odometry_topic_name")
    wheel_velocities_topic_name = LaunchConfiguration("wheel_velocities_topic_name")
    vio_odometry_topic_name = LaunchConfiguration("vio_odometry_topic_name")
    vio_odometry_between_topic_name = LaunchConfiguration("vio_odometry_between_topic_name")
    logging_dir_location = LaunchConfiguration("logging_dir_location")

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
        DeclareLaunchArgument("between_lidar_odometry_topic_name", default_value="/dlio2/odom_node/odom22", description="Between lidar odometry topic name"),
        DeclareLaunchArgument("wheel_odometry_topic_name", default_value="/wheel_odometry", description="Wheel odometry topic name"),
        DeclareLaunchArgument("wheel_velocities_topic_name", default_value="/wheel_velocities", description="Wheel velocities topic name"),
        DeclareLaunchArgument("vio_odometry_topic_name", default_value="/zed/zed_node/pose_with_covariance", description="VIO odometry topic name"),
        DeclareLaunchArgument("vio_odometry_between_topic_name", default_value="/zed/zed_node/odom", description="Between VIO odometry topic name"),
        DeclareLaunchArgument("gnss_topic_name", default_value="/navsatfix", description="GNSS topic name"),
        DeclareLaunchArgument("logging_dir_location", default_value=os.path.join(pkg_dir, "logging"), description="Logging directory location"),

        # Main Node without GNSS
        Node(
            package="b2w_estimator_graph_ros2",
            executable="b2w_estimator_graph_ros2_node",
            name="b2w_estimator_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"launch/optimizationResultLoggingPath": logging_dir_location},
                core_graph_config_param_file,
                trajectory_alignment_param_file,
                b2w_gnss_param_file,
                os.path.join(pkg_dir, "config", "core", "core_graph_params_gnss.yaml"),
                core_extrinsic_param_file,
                os.path.join(pkg_dir, "config", "b2w_specific", "b2w_graph_params_gnss.yaml"),
                b2w_extrinsic_param_file,
            ],
            remappings=[
                ("/imu_topic", imu_topic_name),
                ("/lidar_odometry_topic", lidar_odometry_topic_name),
                ("/between_lidar_odometry_topic", between_lidar_odometry_topic_name),
                ("/wheel_odometry_topic", wheel_odometry_topic_name),
                ("/wheel_velocities_topic", wheel_velocities_topic_name),
                ("/vio_odometry_topic", vio_odometry_topic_name),
                ("/vio_odometry_between_topic", vio_odometry_between_topic_name),
                ("/gnss_topic", gnss_topic_name),
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
            output="log",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["-0.49", "0.0", "0.14", "0", "0", "0", "lidar", "gnss"],
        )

        # Main Node with GNSS
        # Node(
        #     package="b2w_estimator_graph_ros2",
        #     executable="b2w_estimator_graph_ros2_node",
        #     name="b2w_estimator_node",
        #     output="screen",
        #     parameters=[
        #         {"use_sim_time": use_sim_time},
        #         {"launch/optimizationResultLoggingPath": logging_dir_location},
        #         core_graph_config_param_file,
        #         os.path.join(pkg_dir, "config", "core", "core_graph_params_gnss.yaml"),
        #         core_extrinsic_param_file,
        #         os.path.join(pkg_dir, "config", "b2w_specific", "b2w_graph_params_gnss.yaml"),
        #         b2w_extrinsic_param_file,
        #     ],
        #     remappings=[
        #         ("/imu_topic", imu_topic_name),
        #         ("/lidar_odometry_topic", lidar_odometry_topic_name),
        #         ("/between_lidar_odometry_topic", between_lidar_odometry_topic_name),
        #         ("/wheel_odometry_topic", wheel_odometry_topic_name),
        #         ("/wheel_velocities_topic", wheel_velocities_topic_name),
        #         ("/vio_odometry_topic", vio_odometry_topic_name),
        #         ("/gnss_topic", gnss_topic_name),
        #     ],
        #     condition=IfCondition(use_gnss_unary),
        # ),
    ])
