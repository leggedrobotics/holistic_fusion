#!/usr/bin/env python3

# Code taken from: https://gist.github.com/awesomebytes/51470efe54b45045c50263f56d7aec27

import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def filter_topics(in_bag, out_bag, frames_we_want):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=in_bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic.name: topic.type for topic in topic_types}

    writer = rosbag2_py.SequentialWriter()
    writer_storage_options = rosbag2_py.StorageOptions(
        uri=out_bag, storage_id="sqlite3"
    )
    writer.open(writer_storage_options, converter_options)

    for topic in topic_types:
        writer.create_topic(topic)

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == "/tf" and hasattr(msg, "transforms"):
            transforms_to_keep = []
            for transform in msg.transforms:
                if (
                    transform.header.frame_id in frames_we_want
                    and transform.child_frame_id in frames_we_want
                ):
                    transforms_to_keep.append(transform)

            if transforms_to_keep:
                msg.transforms = transforms_to_keep
                writer.write(topic, data, t)

        elif topic != "/tf":
            writer.write(topic, data, t)


def filter_topics_2(in_bag, out_bag, frames_we_dont_want):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=in_bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic.name: topic.type for topic in topic_types}

    writer = rosbag2_py.SequentialWriter()
    writer_storage_options = rosbag2_py.StorageOptions(
        uri=out_bag, storage_id="sqlite3"
    )
    writer.open(writer_storage_options, converter_options)

    for topic in topic_types:
        writer.create_topic(topic)

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == "/tf" and hasattr(msg, "transforms"):
            transforms_to_keep = []
            for transform in msg.transforms:
                if (
                    transform.header.frame_id not in frames_we_dont_want
                    and transform.child_frame_id not in frames_we_dont_want
                ):
                    transforms_to_keep.append(transform)

            if transforms_to_keep:
                msg.transforms = transforms_to_keep
                writer.write(topic, data, t)

        elif topic != "/tf":
            writer.write(topic, data, t)


if __name__ == "__main__":
    print("Starting")
    in_bag = sys.argv[1]
    out_bag = sys.argv[2]

    # Use one of the filtering functions as needed
    # Example:
    # filter_topics(in_bag, out_bag, ['base_link', 'odom', 'map', 'torso', 'Hip', 'Pelvis', 'Tibia', 'base_footprint'])
    filter_topics_2(
        in_bag, out_bag, ["odom_graph_msf", "world_graph_msf", "map_o3d_gmsf"]
    )

    print("Done")
