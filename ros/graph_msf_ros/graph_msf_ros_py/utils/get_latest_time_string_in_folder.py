#!/usr/bin/env python

# Imports
import os

def get_latest_time_string_in_folder(folder_path: str) -> str:
    print(f"All directories located in: {folder_path}:")
    # Load all files in location
    fil_dir_list = os.listdir(folder_path)
    # Only get directories
    dirs = [f for f in fil_dir_list if os.path.isdir(os.path.join(folder_path, f))]
    # We want to only get the latest files
    dirs.sort()
    latest_dir = dirs[-1]
    # Assume that directory name is the time string
    time_string = latest_dir
    return time_string