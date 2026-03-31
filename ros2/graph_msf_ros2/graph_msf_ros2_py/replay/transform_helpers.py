#!/usr/bin/env python3

# General Packages
from scipy.spatial.transform import Rotation as R
import copy
import sys

# ROS 2
import rclpy
from rclpy.time import Time
from rclpy.serialization import serialize_message
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import rosbag2_py


def invert_transform(tf_msg):
    # Old orientation
    q_old = [
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w,
    ]
    # Invert the orientation
    r_old = R.from_quat(q_old)
    r_new = r_old.inv()
    q_new = r_new.as_quat()

    # Old translation
    t_old = [
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z,
    ]
    t_new = -r_new.apply(t_old)

    # Define the new transform
    tf_msg_new = TransformStamped()
    tf_msg_new.header = copy.deepcopy(tf_msg.header)
    tf_msg_new.header.frame_id = tf_msg.child_frame_id
    tf_msg_new.child_frame_id = tf_msg.header.frame_id

    # Set the new translation
    tf_msg_new.transform.translation.x = t_new[0]
    tf_msg_new.transform.translation.y = t_new[1]
    tf_msg_new.transform.translation.z = t_new[2]

    # Set the new orientation
    tf_msg_new.transform.rotation.x = q_new[0]
    tf_msg_new.transform.rotation.y = q_new[1]
    tf_msg_new.transform.rotation.z = q_new[2]
    tf_msg_new.transform.rotation.w = q_new[3]

    return tf_msg_new


def create_transform_stamped(
    timestamp: float,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
    fixed_frame_id: str,
    child_frame_id: str,
):
    t = TransformStamped()
    t.header.stamp = Time(seconds=float(timestamp)).to_msg()
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.x = float(qx)
    t.transform.rotation.y = float(qy)
    t.transform.rotation.z = float(qz)
    t.transform.rotation.w = float(qw)
    t.header.frame_id = fixed_frame_id
    t.child_frame_id = child_frame_id
    return t


def write_bag(input_file_path, output_bag_path, fixed_frame_id, child_frame_id):
    first_line = True
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(
        uri=output_bag_path, storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    # Set the topic information for /tf
    tf_topic_info = rosbag2_py.TopicMetadata(
        name="/tf", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"
    )
    writer.create_topic(tf_topic_info)

    with open(input_file_path, "r") as file:
        for line in file:
            # Header line
            if first_line:
                first_line = False
                continue

            # Assuming format: timestamp,x,y,z,qw,qx,qy,qz
            parts = line.strip().split(",")
            if len(parts) < 11:
                continue  # Skip malformed lines
            timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw = parts
            tf_msg = create_transform_stamped(
                timestamp=timestamp,
                x=x,
                y=y,
                z=z,
                qw=qw,
                qx=qx,
                qy=qy,
                qz=qz,
                fixed_frame_id=fixed_frame_id,
                child_frame_id=child_frame_id,
            )

            # Invert the transform
            inv_tf = invert_transform(tf_msg)

            # Wrap in TFMessage
            tf_message = TFMessage(transforms=[inv_tf])

            # Write to the bag file
            writer.write(
                "/tf",
                serialize_message(tf_message),
                Time(seconds=float(timestamp)).nanoseconds,
            )

    print(f"Conversion complete. Output saved to {output_bag_path}")


if __name__ == "__main__":
    rclpy.init()
    if len(sys.argv) < 5:
        print(
            "Usage: python3 convert_ros2_bag_with_transform.py <input_file> <output_bag> <fixed_frame_id> <child_frame_id>"
        )
        sys.exit(1)

    input_file = sys.argv[1]
    output_bag = sys.argv[2]
    fixed_frame_id = sys.argv[3]
    child_frame_id = sys.argv[4]

    write_bag(input_file, output_bag, fixed_frame_id, child_frame_id)
    rclpy.shutdown()
