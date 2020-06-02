#!/usr/bin/env python
# Copyright (c) 2019 Lucas Walter
# subscribe to a Float32 message (later support other number types and fields)
# and publish the average and standard deviation given a window size (in number of message or later time)

import rospy
import rostopic

# from roslib.message import get_message_class
# from rospy.msg import AnyMsg
from std_msgs.msg import Float32


class TopicState(object):
    def __init__(self):
        self.last_msg = None
        self.float_sub = rospy.Subscriber('topic', Float32, self.callback, queue_size=10)
        self.diff_pub = rospy.Publisher('topic_diff', Float32, queue_size=10)

    def callback(self, msg):
        if self.last_msg is not None:
            diff = msg.data - self.last_msg.data
            self.diff_pub.publish(Float32(diff))
        self.last_msg = msg


if __name__ == '__main__':
    rospy.init_node('topic_diff')
    topic_state = TopicState()
    rospy.spin()
