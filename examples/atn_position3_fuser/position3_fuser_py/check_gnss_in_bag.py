import rosbag
import sys


def get_bag_start_time(bag_file_path):
    try:
        with rosbag.Bag(bag_file_path, 'r') as bag:
            start_time = bag.get_start_time()
            print(f"The start time of the bag is: {start_time} seconds (Unix epoch time).")
            return start_time
    except rosbag.bag.ROSBagException as e:
        print(f"Error reading the rosbag file: {e}")
    except FileNotFoundError:
        print("The specified rosbag file was not found.")

def check_topic_in_rosbag(bag_file_path, topic_name):
    try:
        # Open the bag file
        with rosbag.Bag(bag_file_path, 'r') as bag:
            # Check if the topic is present
            topic_first_appearance = None
            for topic, msg, t in bag.read_messages():
                if topic == topic_name:
                    topic_first_appearance = t.to_sec()
                    break

            if topic_first_appearance is not None:
                print(f"The topic '{topic_name}' first appears at {topic_first_appearance} seconds in the bag.")
                return topic_first_appearance
            else:
                print(f"The topic '{topic_name}' is not present in the bag.")
                return None

    except rosbag.bag.ROSBagException as e:
        print(f"Error reading the rosbag file: {e}")
    except FileNotFoundError:
        print("The specified rosbag file was not found.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python check_topic_in_rosbag.py <bag_file_path>")
    else:
        bag_file_path = sys.argv[1]
        topic_name = "/gt_box/inertial_explorer/tc/odometry"
        # Check when the topic first appears in the bag
        topic_first_appearance = check_topic_in_rosbag(bag_file_path, topic_name)
        if topic_first_appearance is None:
            print("The topic is not present in the bag.")
            sys.exit(1)
        # Get the start time of the bag
        bag_start_time = get_bag_start_time(bag_file_path)
        # Check
        if bag_start_time is None or topic_first_appearance < bag_start_time:
            print("Error computing relative time.")
            sys.exit(1)

        # Compute relative time of first appearance of the topic
        relative_time = topic_first_appearance - bag_start_time
        print(f"The topic '{topic_name}' first appears at {relative_time} seconds after the start of the bag.")

        # If it appears within the first 10 seconds, return exit code 0
        if relative_time < 10:
            print("The topic appears within the first 10 seconds, hence the test is successful.")
            sys.exit(0)
        else:
            print("The topic does not appear within the first 10 seconds, hence the test is unsuccessful.")
            sys.exit(1)
