#!/usr/bin/env python3

# General Packages
import os
import matplotlib.pyplot as plt
import numpy as np
import sys

# Path to the folder containing the files
# HOME_DIR = str(os.path.expanduser("~"))
# dir_path = os.path.join(
#     HOME_DIR,
#     "workspaces/rsl_workspaces/graph_msf_ws/src/graph_msf_dev/examples/anymal_estimator_graph/logging",
# )

# Add the parent directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Workspace
import utils.get_latest_time_string_in_folder as get_latest_time_string_in_folder


def plot_quantities_in_file(
        file_path: str, type: str, dir_path: str, latest_file_string: str
):
    # Data
    fields = []
    # Bias
    if "bias" in type:
        fields = ["t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"]
    # Covariance --> Then not a transform or pose
    elif "covariance" in type:
        fields = ["t",
                  "cov_00", "cov_01", "cov_02", "cov_03", "cov_04", "cov_05",
                  "cov_10", "cov_11", "cov_12", "cov_13", "cov_14", "cov_15",
                  "cov_20", "cov_21", "cov_22", "cov_23", "cov_24", "cov_25",
                  "cov_30", "cov_31", "cov_32", "cov_33", "cov_34", "cov_35",
                  "cov_40", "cov_41", "cov_42", "cov_43", "cov_44", "cov_45",
                  "cov_50", "cov_51", "cov_52", "cov_53", "cov_54", "cov_55"]
    elif "transform" in type or "pose" in type:
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
    print(f"Fields: {fields}")
    #print("First 5 rows of data: ", data[:5, :])
    # Sorty by time
    # indices = np.argsort(data[:, 0])
    # data = data[indices, :]
    # print(indices)
    # Plot
    plt.figure()
    if "covariance" not in type:
        for i, field in enumerate(fields):
            # Do not plot time field
            if i == 0 or field == "qw" or field == "qx" or field == "qy" or field == "qz":
                continue
            else:
                print(f"Plotting field: {field}")
                # Create figure
                plt.plot(data[:, 0], data[:, i], label=field)

            # Create plot when entering last field for each type
            if "bias" in type and field == "a_z":
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
                plt.figure()
            elif ("transform" in type or "pose" in type) and field == "z":
                plt.xlabel("Time [s]")
                plt.ylabel("Position [m]")
                plt.title(f"{type} Position Over Time")
                plt.legend()
                plt.grid()
                # Save the plot
                plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_position.png")
                plt.savefig(plot_path)
                plt.figure()
            elif ("transform" in type or "pose" in type) and field == "yaw":
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
                plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}.png")
                plt.savefig(plot_path)
                plt.figure()
            else:  # Continue plotting
                continue

            print(f"Plot saved to: {plot_path}")

    # Now get the covariance and x, y plot
    plt.figure()
    # Covariance
    if "covariance" in type:
        # Get the diagonal elements
        plt.plot(data[:, 0], data[:, 1], label="cov_xx")
        plt.plot(data[:, 0], data[:, 8], label="cov_yy")
        plt.plot(data[:, 0], data[:, 15], label="cov_zz")
        #
        plt.xlabel("Time [s]")
        plt.ylabel("Covariance [m]")
        plt.title(f"{type} Covariance Over Time")
        plt.legend()
        plt.grid()
        # Save the plot
        plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}.png")
        plt.savefig(plot_path)
        print(f"Covariance plot saved to: {plot_path}")
    # X, Y
    elif "transform" in type or "pose" in type:
        # Data
        plt.plot(data[:, 1], data[:, 2], label="x-y")
        # Plot
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f"{type} x-y Position Over Time")
        plt.legend()
        plt.grid()
        # Save the plot
        plot_path = os.path.join(dir_path, f"{latest_file_string}_plot_{type}_x_y_position.png")
        plt.savefig(plot_path)
        print(f"X-Y-plot saved to: {plot_path}")


def main():
    # Get Directory Path as argument
    if len(sys.argv) > 1:
        ros_package_name = sys.argv[1]
        print(f"ROS package name: {ros_package_name}")
    else:
        print(f"ROS package name not provided. Exiting...")
        exit()

    # Find the directory path
    import roslib
    dir_path = os.path.join(roslib.packages.get_pkg_dir(ros_package_name), "logging")
    print(f"Directory path: {dir_path}")

    # We want to only get the latest files
    latest_file_string = (
        get_latest_time_string_in_folder.get_latest_time_string_in_folder(dir_path)
    )
    print("Latest file string: ", latest_file_string)
    # Plot following files:
    file_types = [
        "B_imu_bias",
        "R_6D_transform",
        "v_state_3D_velocity",
        "X_state_6D_pose",
        "X_state_6D_pose_covariance",
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
