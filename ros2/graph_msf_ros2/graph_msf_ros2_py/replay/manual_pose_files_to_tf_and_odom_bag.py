#!/usr/bin/env python

# Imports
import os.path
from pathlib import Path

HOME_DIR = str(Path.home())

# Workspace
from graph_msf_ros2_py.replay.transform_helpers import write_bag
from graph_msf_ros2_py.utils.get_latest_time_string_in_folder import (
    get_latest_time_string_in_folder,
)

# Input and output files
dir_path = os.path.join(
    HOME_DIR,
    "workspaces/rsl_workspaces/graph_msf_ws/src/graph_msf_dev/examples/anymal_estimator_graph/logging",
)
latest_time_string = get_latest_time_string_in_folder(folder_path=dir_path)
input_file_path = os.path.join(
    dir_path, latest_time_string + "_optimized_X_state_transform.csv"
)
output_bag_path = os.path.join(
    dir_path, latest_time_string + "_bag_X_state_transform.bag"
)
fixed_frame_id = "offline_world_graph_msf"
child_frame_id = "imu_link"


def main():
    # Go through the input file and write the data to the output bag
    write_bag(input_file_path, output_bag_path, fixed_frame_id, child_frame_id)


if __name__ == "__main__":
    main()
