#!/usr/bin/env python3

# General Packages
import os
import matplotlib.pyplot as plt
import numpy as np
import sys
from ament_index_python.packages import get_package_share_directory
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Path to the folder containing the files
HOME_DIR = str(os.path.expanduser("~"))
dir_path = os.path.join(
    HOME_DIR,
    "workspaces/rsl_workspaces/graph_msf_ws/src/graph_msf_dev/examples/anymal_estimator_graph/logging",
)

# Add the parent directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Workspace
import utils.get_latest_time_string_in_folder as get_latest_time_string_in_folder


def plot_quantities_in_file(file_path: str, type: str, dir_path: str, latest_file_string: str):
    try:
        # Data
        fields = []
        # Bias
        if "bias" in type:
            fields = ["t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"]
        # Covariance
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

        # Read data
        data = []
        first_line = True
        with open(file_path, "r") as file:
            type = type.split(".")[0]  # Remove everything after the dot
            for line in file:
                if first_line:
                    first_line = False
                    continue
                parts = line.strip().split(",")
                if len(parts) < len(fields):
                    logger.warning(f"Skipping malformed line: {line}")
                    continue
                data.append(np.asarray(parts))

        if not data:
            logger.error(f"No valid data found in {file_path}")
            return

        # Process Data
        data = np.array(data, dtype=np.float64)
        data[:, 0] = data[:, 0] - data[0, 0]  # Subtract first timestamp
        logger.info(f"Data shape: {data.shape}")
        logger.info(f"Fields: {fields}")

        # Plot
        plt.figure()
        if "covariance" not in type:
            for i, field in enumerate(fields):
                if i == 0 or field in ["qw", "qx", "qy", "qz"]:
                    continue
                
                logger.info(f"Plotting field: {field}. Max: {np.max(data[:, i])}. Min: {np.min(data[:, i])}")
                plt.plot(data[:, 0], data[:, i], label=field)

                # Create plot when entering last field for each type
                if "bias" in type and field == "a_z":
                    plt.xlabel("Time [s]")
                    plt.ylabel("Acceleration [m/s^2]")
                    plt.title(f"{type} Acceleration Over Time")
                    plt.legend()
                    plt.grid()
                    plot_path = os.path.join(dir_path, f"{latest_file_string}{type}_acceleration.png")
                    plt.savefig(plot_path)
                    plt.figure()
                elif "bias" in type and field == "w_z":
                    plt.xlabel("Time [s]")
                    plt.ylabel("Angular Velocity [rad/s]")
                    plt.title(f"{type} Angular Velocity Over Time")
                    plt.legend()
                    plt.grid()
                    plot_path = os.path.join(dir_path, f"{latest_file_string}{type}_angular_velocity.png")
                    plt.savefig(plot_path)
                elif ("transform" in type or "pose" in type) and field == "z":
                    plt.xlabel("Time [s]")
                    plt.ylabel("Position [m]")
                    plt.title(f"{type} Position Over Time")
                    plt.legend()
                    plt.grid()
                    plot_path = os.path.join(dir_path, f"{latest_file_string}{type}_position.png")
                    plt.savefig(plot_path)
                    plt.figure()
                elif ("transform" in type or "pose" in type) and field == "yaw":
                    plt.xlabel("Time [s]")
                    plt.ylabel("Orientation [rad]")
                    plt.title(f"{type} Orientation Over Time")
                    plt.legend()
                    plt.grid()
                    plot_path = os.path.join(dir_path, f"{latest_file_string}{type}_orientation.png")
                    plt.savefig(plot_path)
                    plt.figure()
                elif "velocity" in type and field == "v_z":
                    plt.xlabel("Time [s]")
                    plt.ylabel("Velocity [m/s]")
                    plt.title(f"{type} Velocity Over Time")
                    plt.legend()
                    plt.grid()
                    plot_path = os.path.join(dir_path, f"{latest_file_string}{type}.png")
                    plt.savefig(plot_path)
                    plt.figure()

                logger.info(f"Plot saved to: {plot_path}")

        # Covariance and x-y plots
        if "covariance" in type:
            plt.figure()
            plt.plot(data[:, 0], data[:, 1], label="cov_xx")
            plt.plot(data[:, 0], data[:, 8], label="cov_yy")
            plt.plot(data[:, 0], data[:, 15], label="cov_zz")
            plt.xlabel("Time [s]")
            plt.ylabel("Covariance [m]")
            plt.title(f"{type} Covariance Over Time")
            plt.legend()
            plt.grid()
            plot_path = os.path.join(dir_path, f"{latest_file_string}{type}.png")
            plt.savefig(plot_path)
            logger.info(f"Covariance plot saved to: {plot_path}")
        elif "transform" in type or "pose" in type:
            plt.figure()
            plt.plot(data[:, 1], data[:, 2], label="x-y")
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.title(f"{type} x-y Position Over Time")
            plt.axis("equal")
            plt.legend()
            plt.grid()
            plot_path = os.path.join(dir_path, f"{latest_file_string}{type}_x_y_position.png")
            plt.savefig(plot_path)
            logger.info(f"X-Y plot saved to: {plot_path}")

    except Exception as e:
        logger.error(f"Error plotting {type}: {str(e)}")
        raise

def main():
    try:
        if len(sys.argv) != 2:
            logger.error("Usage: python3 plot_latest_quantitites_in_folder.py <package_name>")
            sys.exit(1)

        package_name = sys.argv[1]
        logger.info(f"Package name: {package_name}")

        # Get package directory
        try:
            package_dir = get_package_share_directory(package_name)
            dir_path = os.path.join(package_dir, "logging")
            logger.info(f"Directory path: {dir_path}")
        except Exception as e:
            logger.error(f"Failed to find package {package_name}: {str(e)}")
            sys.exit(1)

        # Get latest directory
        latest_dir_string = get_latest_time_string_in_folder.get_latest_time_string_in_folder(dir_path)
        logger.info(f"Latest directory: {latest_dir_string}")

        # File types to plot
        file_types = [
            "B_imu_bias",
            "R_6D_transform",
            "v_state_3D_velocity",
            "X_state_6D_pose",
            "X_state_6D_pose_covariance",
        ]

        # Get files
        files = os.listdir(os.path.join(dir_path, latest_dir_string))
        files = [f for f in files if f.endswith(".csv")]
        logger.info(f"Files in {latest_dir_string}: {files}")

        # Plot files
        for file in files:
            for file_type in file_types:
                if file_type in file:
                    logger.info(f"Plotting {file_type} from {file}")
                    plot_quantities_in_file(
                        file_path=os.path.join(dir_path, latest_dir_string, file),
                        type=file,
                        dir_path=os.path.join(dir_path, latest_dir_string),
                        latest_file_string="",
                    )

    except Exception as e:
        logger.error(f"Error in main: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    logger.info("Starting")
    main()
    logger.info("Done")
