#!/usr/bin/env python3

# Code taken from: https://gist.github.com/awesomebytes/51470efe54b45045c50263f56d7aec27

import rosbag
import sys

def filter_rosbag(input_bag_path, output_bag_path):
    # Open the original rosbag file
    with rosbag.Bag(input_bag_path, 'r') as input_bag:
        # Create a new rosbag file for the filtered data
        with rosbag.Bag(output_bag_path, 'w') as output_bag:
            # Loop through each topic and message in the input rosbag
            for topic, msg, t in input_bag.read_messages():
                # Check if the current message's topic is not '/tf'
                if topic != '/tf':
                    # Write the message to the output bag if it's not '/tf'
                    output_bag.write(topic, msg, t)

if __name__ == '__main__':
    print("Starting")
    in_bag = sys.argv[1]
    out_bag = sys.argv[2]
    # filter_topics(in_bag, out_bag, ['base_link', 'odom', 'map',
    #                                 'torso', 'Hip', 'Pelvis', 'Tibia', 'base_footprint'])
    filter_rosbag(input_bag_path=in_bag, output_bag_path=out_bag)
    print("Done")