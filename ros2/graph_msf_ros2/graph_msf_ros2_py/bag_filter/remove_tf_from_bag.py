#!/usr/bin/env python3

# Code taken from: https://gist.github.com/awesomebytes/51470efe54b45045c50263f56d7aec27

import sys
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def filter_rosbag(input_bag_path, output_bag_path):
    # Initialize the rosbag2 reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=input_bag_path, storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # Get all topics and types from the input bag
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Initialize the rosbag2 writer
    writer = rosbag2_py.SequentialWriter()
    writer_storage_options = rosbag2_py.StorageOptions(
        uri=output_bag_path, storage_id="sqlite3"
    )
    writer.open(writer_storage_options, converter_options)

    # Create topics in the output bag based on the input bag's topics
    for topic in topic_types:
        writer.create_topic(topic)

    # Loop through each topic and message in the input rosbag
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # Skip the /tf topic
        if topic == "/tf":
            continue

        # Deserialize and re-serialize the message for writing
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        writer.write(topic, serialize_message(msg), timestamp)


if __name__ == "__main__":
    print("Starting")
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    filter_rosbag(input_bag_path=input_bag, output_bag_path=output_bag)
    print("Done")
