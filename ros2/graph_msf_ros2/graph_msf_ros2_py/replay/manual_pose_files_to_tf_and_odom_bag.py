#!/usr/bin/env python3

import os.path
import sys
import logging
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Add the parent directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Workspace
from graph_msf_ros2_py.replay.transform_helpers import write_bag
from graph_msf_ros2_py.utils.get_latest_time_string_in_folder import get_latest_time_string_in_folder

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def process_file(dir_path: str, csv_file_name: str, fixed_frame_id: str, child_frame_id: str):
    try:
        latest_time_string = get_latest_time_string_in_folder(folder_path=dir_path)
        input_file_path = os.path.join(dir_path, latest_time_string, f"{csv_file_name}.csv")
        output_bag_path = os.path.join(dir_path, latest_time_string, f"bag_{csv_file_name}.bag")

        logger.info(f"Processing file: {input_file_path}")
        logger.info(f"Output bag: {output_bag_path}")
        
        write_bag(
            input_file_path=input_file_path,
            output_bag_path=output_bag_path,
            fixed_frame_id=fixed_frame_id,
            child_frame_id=child_frame_id
        )
        logger.info(f"Successfully processed {csv_file_name}")
    except Exception as e:
        logger.error(f"Error processing {csv_file_name}: {str(e)}")
        raise

def main():
    try:
        # Get command line arguments
        if len(sys.argv) != 3:
            logger.error("Usage: python3 manual_pose_files_to_tf_and_odom_bag.py <package_name> <imu_frame_name>")
            sys.exit(1)

        package_name = sys.argv[1]
        imu_frame_name = sys.argv[2]

        # Validate IMU frame name
        valid_frames = ["cpt7_imu", "imu_link"]
        if imu_frame_name not in valid_frames:
            logger.error(f"Invalid IMU frame name. Must be one of: {valid_frames}")
            sys.exit(1)

        logger.info(f"Package name: {package_name}")
        logger.info(f"IMU frame name: {imu_frame_name}")

        # Get package directory
        try:
            package_dir = get_package_share_directory(package_name)
            dir_path = os.path.join(package_dir, "logging")
            logger.info(f"Directory path: {dir_path}")
        except Exception as e:
            logger.error(f"Failed to find package {package_name}: {str(e)}")
            sys.exit(1)

        # Process files
        files_to_process = [
            "X_state_6D_pose",
            "rt_world_x_state_6D_pose",
            "optimized_X_state_transform"  # Additional file from ROS2 version
        ]

        fixed_frame_id = "gt_world"  # or "offline_world_graph_msf" depending on your needs

        for file_name in files_to_process:
            try:
                process_file(dir_path, file_name, fixed_frame_id, imu_frame_name)
            except Exception as e:
                logger.warning(f"Skipping {file_name} due to error: {str(e)}")
                continue

    except Exception as e:
        logger.error(f"Error in main: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    logger.info("Starting")
    main()
    logger.info("Done")
