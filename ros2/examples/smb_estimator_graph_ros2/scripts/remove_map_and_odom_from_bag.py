#!/usr/bin/env python3

# Code taken from: https://gist.github.com/awesomebytes/51470efe54b45045c50263f56d7aec27

import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosbag2_py import SequentialWriter, StorageOptions as WriterStorageOptions
from tf2_msgs.msg import TFMessage
import yaml

def filter_topics(in_bag, out_bag, frames_we_want):
    # Setup reader
    storage_options = StorageOptions(uri=in_bag, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Setup writer
    writer_storage_options = WriterStorageOptions(uri=out_bag, storage_id='sqlite3')
    writer = SequentialWriter()
    writer.open(writer_storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_info.name: topic_info.type for topic_info in topic_types}

    print("Writing to " + out_bag)
    print("Reading from " + in_bag)

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_type)

        if topic_name == "/tf" and isinstance(msg, TFMessage):
            transforms_to_keep = []
            for transform in msg.transforms:
                if transform.header.frame_id in frames_we_want and transform.child_frame_id in frames_we_want:
                    transforms_to_keep.append(transform)
            msg.transforms = transforms_to_keep
            writer.write(topic_name, data, timestamp)
        elif topic_name != '/tf':
            writer.write(topic_name, data, timestamp)

def filter_topics_2(in_bag, out_bag, frames_we_dont_want):
    # Setup reader
    storage_options = StorageOptions(uri=in_bag, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Setup writer
    writer_storage_options = WriterStorageOptions(uri=out_bag, storage_id='sqlite3')
    writer = SequentialWriter()
    writer.open(writer_storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_info.name: topic_info.type for topic_info in topic_types}

    print("Writing to " + out_bag)
    print("Reading from " + in_bag)

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_type)

        if topic_name == "/tf" and isinstance(msg, TFMessage):
            transforms_to_keep = []
            for transform in msg.transforms:
                if transform.header.frame_id not in frames_we_dont_want and transform.child_frame_id not in frames_we_dont_want:
                    transforms_to_keep.append(transform)
            msg.transforms = transforms_to_keep
            writer.write(topic_name, data, timestamp)
        elif topic_name != '/tf':
            writer.write(topic_name, data, timestamp)

if __name__ == '__main__':
    print("Starting")
    in_bag = sys.argv[1]
    out_bag = sys.argv[2]
    # filter_topics(in_bag, out_bag, ['base_link', 'odom', 'map',
    #                                 'torso', 'Hip', 'Pelvis', 'Tibia', 'base_footprint'])
    filter_topics_2(in_bag, out_bag, ['odom'])
    print("Done")