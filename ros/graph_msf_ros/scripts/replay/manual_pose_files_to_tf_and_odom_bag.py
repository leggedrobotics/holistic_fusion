#!/usr/bin/env python
import os.path
from pathlib import Path

HOME_DIR = str(Path.home())

# Workspace
from transform_helpers import write_bag

# Input and output files
dir_path = os.path.join(
    HOME_DIR,
    "workspaces/rsl_workspaces/graph_msf_ws/src/graph_msf_dev/examples/anymal_estimator_graph/logging",
)
input_file_path = os.path.join(
    dir_path, "Wed_Jul__3_13:33:09_2024_optimized_X_state_transform.csv"
)
output_bag_path = os.path.join(
    dir_path, "Wed_Jul__3_13:33:09_2024_optimized_X_state_transform.bag"
)
fixed_frame_id = "offline_world_graph_msf"
child_frame_id = "imu_link"


def main():
    # Go through the input file and write the data to the output bag
    write_bag(input_file_path, output_bag_path, fixed_frame_id, child_frame_id)



if __name__ == "__main__":
    main()
