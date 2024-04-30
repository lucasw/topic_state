#!/usr/bin/env python
"""
Copyright 2024 Lucas Walter

Look at every message with a header in a bag and computer per-topic statistics
for the age of the messages
"""

import sys

import numpy as np
import rosbag
import rospy


def main():
    rospy.init_node("bag_msg_age")
    argv = [arg for arg in sys.argv[1:] if not arg.startswith("_")]
    bags = [arg for arg in argv if arg.endswith(".bag")]

    ages_by_topic = {}

    no_header_topics = {}

    for bag_name in bags:
        rospy.loginfo(bag_name)
        with rosbag.Bag(bag_name) as bag:
            for (topic, message, rx_stamp) in bag.read_messages():
                if rospy.is_shutdown():
                    sys.exit(0)
                if topic in no_header_topics:
                    continue
                if not hasattr(message, "header"):
                    no_header_topics[topic] = True
                    rospy.loginfo(f"not using {topic}, no header preset")
                    continue

                header_stamp = message.header.stamp
                elapsed = rx_stamp - header_stamp

                if topic not in ages_by_topic:
                    rospy.loginfo(topic)
                    ages_by_topic[topic] = []
                ages_by_topic[topic].append(elapsed.to_sec())

    print()
    rospy.loginfo("topic len min mean max")
    for topic, elapsed_list in ages_by_topic.items():
        elapsed = np.array(elapsed_list)
        text = f"{topic} {len(elapsed_list)}"
        text += f" {np.min(elapsed):0.3f}"
        text += f" {np.mean(elapsed):0.3f}"
        elapsed_max = np.max(elapsed)
        text += f" {elapsed_max:0.3f}"
        if elapsed_max < 0.05:
            rospy.loginfo(text)
        elif elapsed_max < 1.0:
            rospy.logwarn(text)
        else:
            rospy.logerr(text)


if __name__ == "__main__":
    main()
