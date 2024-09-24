#!/usr/bin/env python

# Imports
import os.path
import sys

# Add the parent directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Workspace
from replay.transform_helpers import write_bag
from utils.get_latest_time_string_in_folder import get_latest_time_string_in_folder


if __name__ == "__main__":
    # Get Directory Path as argument
    if len(sys.argv) == 3:
        ros_package_name = sys.argv[1]
        print(f"ROS package name: {ros_package_name}")
        imu_frame_name = sys.argv[2]
        print(f"IMU frame name: {imu_frame_name}")
    elif len(sys.argv) > 1:
        print(f"Too many arguments provided. Expecting [ros_package_name, imu_frame_name]. Exiting...")
        exit()
    else:
        print(f"Not enough arguments provided. Expecting [ros_package_name, imu_frame_name]. Exiting...")
        exit()

    # Find the directory path
    import roslib

    dir_path = os.path.join(roslib.packages.get_pkg_dir(ros_package_name), "logging")
    print(f"Directory path: {dir_path}")

    # Process data
    latest_time_string = get_latest_time_string_in_folder(folder_path=dir_path)
    input_file_path = os.path.join(
        dir_path, latest_time_string + "_X_state_6D_pose.csv"
    )
    output_bag_path = os.path.join(
        dir_path, latest_time_string + "_bag_X_state_6D_pose.bag"
    )
    fixed_frame_id = "offline_world"

    # Call main function
    write_bag(input_file_path=input_file_path, output_bag_path=output_bag_path, fixed_frame_id=fixed_frame_id,
              child_frame_id=imu_frame_name)
