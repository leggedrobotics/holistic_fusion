#!/usr/bin/env python

# General Packages
import os
import matplotlib.pyplot as plt
import numpy as np

# Path to the folder containing the files
HOME_DIR = str(os.path.expanduser("~"))
dir_path = os.path.join(
    HOME_DIR,
    "workspaces/rsl_workspaces/graph_msf_ws/src/graph_msf_dev/examples/anymal_estimator_graph/logging",
)

# Workspace
import graph_msf_ros_py.utils.get_latest_time_string_in_folder as get_latest_time_string_in_folder


def plot_quantities_in_file(
    file_path: str, type: str, dir_path: str, latest_file_string: str
):
    # Data
    fields = []
    if "bias" in type:
        fields = ["t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"]
    elif "transform" in type:
        fields = ["t", "x", "y", "z", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw"]
    elif "velocity" in type:
        fields = ["t", "v_x", "v_y", "v_z"]

    # Read
    data = []
    first_line = True
    with open(file_path, "r") as file:
        for line in file:
            # Header file
            if first_line:
                first_line = False
                continue
            # Assuming your file format is: timestamp,x,y,z,qw,qx,qy,qz
            parts = line.strip().split(",")

            # Write to data
            if len(parts) < len(fields):
                continue  # Skip malformed lines
            data.append(np.asarray(parts))

    # Process Data
    data = np.array(data, dtype=np.float64)
    # Subtract first timestamp
    data[:, 0] = data[:, 0] - data[0, 0]
    print(f"Data shape: {data.shape}")
    print("First 5 rows of data: ", data[:5, :])
    # Sorty by time
    # indices = np.argsort(data[:, 0])
    # data = data[indices, :]
    # print(indices)
    # Plot
    plt.figure()
    for i, field in enumerate(fields):
        # Do not plot time field
        if i == 0 or field == "qw" or field == "qx" or field == "qy" or field == "qz":
            continue
        else:
            print(f"Plotting field: {field}")
            # Create figure
            plt.plot(data[:, 0], data[:, i], label=field)

        # Create plot
        if "transform" in type and field == "z":
            plt.xlabel("Time [s]")
            plt.ylabel("Position [m]")
            plt.title(f"{type} Position Over Time")
            plt.legend()
            plt.grid()
            # Save the plot
            plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_position.png")
            plt.savefig(plot_path)
            plt.figure()
        elif "transform" in type and field == "yaw":
            plt.xlabel("Time [s]")
            plt.ylabel("Orientation [rad]")
            plt.title(f"{type} Orientation Over Time")
            plt.legend()
            plt.grid()
            # Save the plot
            plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_orientation.png")
            plt.savefig(plot_path)
            plt.figure()
        elif "velocity" in type and field == "v_z":
            plt.xlabel("Time [s]")
            plt.ylabel("Velocity [m/s]")
            plt.title(f"{type} Velocity Over Time")
            plt.legend()
            plt.grid()
            # Save the plot
            plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_velocity.png")
            plt.savefig(plot_path)
        elif "bias" in type and field == "a_z":
            plt.xlabel("Time [s]")
            plt.ylabel("Acceleration [m/s^2]")
            plt.title(f"{type} Acceleration Over Time")
            plt.legend()
            plt.grid()
            # Save the plot
            plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_acceleration.png")
            plt.savefig(plot_path)
            plt.figure()
        elif "bias" in type and field == "w_z":
            plt.xlabel("Time [s]")
            plt.ylabel("Angular Velocity [rad/s]")
            plt.title(f"{type} Angular Velocity Over Time")
            plt.legend()
            plt.grid()
            # Save the plot
            plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_angular_velocity.png")
            plt.savefig(plot_path)
        else: # Continue plotting
            continue

        print(f"Plot saved to: {plot_path}")

    # Now get the x, y plot
    if "transform" in type:
        plt.plot(data[:, 1], data[:, 2], label="x-y")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f"{type} x-y Position Over Time")
        plt.legend()
        plt.grid()
        # Save the plot
        plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_x_y_position.png")
        plt.savefig(plot_path)
        print(f"Plot saved to: {plot_path}")



def main():
    # We want to only get the latest files
    latest_file_string = (
        get_latest_time_string_in_folder.get_latest_time_string_in_folder(dir_path)
    )
    print("Latest file string: ", latest_file_string)
    # Plot following files:
    file_types = [
        "B_bias",
        "T_align_transform",
        "V_state_velocity",
        "X_state_transform",
    ]
    # Get all files in the folder that contain the latest file string
    files = os.listdir(dir_path)
    files = [f for f in files if (latest_file_string in f and f.endswith(".csv"))]
    print(f"Files containing {latest_file_string}: {files}")

    # Iterate through files and check if they contain the file types
    for file in files:
        for file_type in file_types:
            if file_type in file:
                print(f"Plotting {file_type} from file: {file}")
                # Plot the quantities in the file
                plot_quantities_in_file(
                    os.path.join(dir_path, file),
                    file_type,
                    dir_path,
                    latest_file_string,
                )


if __name__ == "__main__":
    print("Starting")
    main()
    print("Done")
