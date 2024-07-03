#!/usr/bin/env python

# Imports
import os

def get_latest_time_string_in_folder(folder_path: str) -> str:
    print(f"All files located at: {folder_path}:")
    # Load all files in location
    files = os.listdir(folder_path)
    # Only get csv files
    files = [f for f in files if f.endswith(".csv")]
    # We want to only get the latest files
    files.sort()
    latest_filename = files[-1]
    # Get first part of string before "_optimized"
    time_string = latest_filename.split("_optimized")[0]
    return time_string