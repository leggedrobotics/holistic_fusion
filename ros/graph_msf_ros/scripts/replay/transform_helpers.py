#!/usr/bin/env python# General Packages

from scipy.spatial.transform import Rotation as R
import copy

# ROS
import rospy
from geometry_msgs.msg import TransformStamped
import rosbag
from tf.msg import tfMessage


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

    # Invert the orientation
    r_new = r_old.inv()
    q_new = r_new.as_quat()

    # Old translation
    t_old = [
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z,
    ]

    # New translation
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
    t.header.stamp = rospy.Time.from_sec(float(timestamp))
    # Map to imu
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.x = float(qx)
    t.transform.rotation.y = float(qy)
    t.transform.rotation.z = float(qz)
    t.transform.rotation.w = float(qw)
    # Set the frame IDs
    t.header.frame_id = fixed_frame_id
    t.child_frame_id = child_frame_id
    return t

def write_bag(input_file_path, output_bag_path, fixed_frame_id, child_frame_id):
    first_line = True
    with open(input_file_path, "r") as file, rosbag.Bag(output_bag_path, "w") as bag:
        for line in file:
            # Header file
            if first_line:
                first_line = False
                continue
            # Assuming your file format is: timestamp,x,y,z,qw,qx,qy,qz
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

            # Invert the transform to get the correct orientation
            inv_tf = invert_transform(tf_msg)

            # Wrap the TransformStamped in a tfMessage
            tfm = tfMessage([inv_tf])
            bag.write("/tf", tfm, tf_msg.header.stamp)

    print(f"Conversion complete. Output saved to {output_bag_path}")
