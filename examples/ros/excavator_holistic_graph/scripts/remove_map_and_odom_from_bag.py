#!/usr/bin/env python3

# Code taken from: https://gist.github.com/awesomebytes/51470efe54b45045c50263f56d7aec27

import rosbag
import sys

def filter_topics(in_bag, out_bag, frames_we_want):
    with rosbag.Bag(out_bag, 'w') as outbag:
        print("Writing to " + out_bag)
        print("Reading from " + in_bag)
        for topic, msg, t in rosbag.Bag(in_bag).read_messages():
            if topic == "/tf" and msg.transforms:
                transforms_to_keep = []
                for i in range(len(msg.transforms)):
                    # if its one of the frames we want we keep it
                    if msg.transforms[i].header.frame_id in frames_we_want and msg.transforms[i].child_frame_id in frames_we_want:
                        transforms_to_keep.append(msg.transforms[i])
                        #print("Keeping: " + str(msg.transforms[i]))
                    # else:
                    #     print("Discarding: " + str(msg.transforms[i]))

                msg.transforms = transforms_to_keep
                outbag.write(topic, msg, t)
            elif topic != '/tf':
                outbag.write(topic, msg, t)

def filter_topics_2(in_bag, out_bag, frames_we_dont_want):
    with rosbag.Bag(out_bag, 'w') as outbag:
        print("Writing to " + out_bag)
        print("Reading from " + in_bag)
        for topic, msg, t in rosbag.Bag(in_bag).read_messages():
            if topic == "/tf" and msg.transforms:
                transforms_to_keep = []
                for i in range(len(msg.transforms)):
                    # if its one of the frames we want we keep it
                    if msg.transforms[i].header.frame_id not in frames_we_dont_want and msg.transforms[i].child_frame_id not in frames_we_dont_want:
                        transforms_to_keep.append(msg.transforms[i])
                        #print("Keeping: " + str(msg.transforms[i]))
                    # else:
                    #     print("Discarding: " + str(msg.transforms[i]))

                msg.transforms = transforms_to_keep
                outbag.write(topic, msg, t)
            elif topic != '/tf':
                outbag.write(topic, msg, t)


if __name__ == '__main__':
    print("Starting")
    in_bag = sys.argv[1]
    out_bag = sys.argv[2]
    # filter_topics(in_bag, out_bag, ['base_link', 'odom', 'map',
    #                                 'torso', 'Hip', 'Pelvis', 'Tibia', 'base_footprint'])
    filter_topics_2(in_bag, out_bag, ['map', 'odom'])
    print("Done")