#!/usr/bin/env python
# Copyright (c) 2022 Lucas Walter
#
# subscribe to multiple messages and echo them all, interleave their output
# TODO(lucasw) allow other than 2 topics
# Also add exact and approximate time sync options

import sys

import rospy
from roslib.message import get_message_class


class MultiEcho(object):
    def __init__(self):
        # TODO(lucasw) support many topics in param list
        # for now just one

        self.max_len = rospy.get_param("~max_len", 200)

        self.topics = {}
        self.topic_types = {}
        self.subs = {}
        for index in range(2):
            self.topics[index] = rospy.get_param(f"~t{index}")
            self.subs[index] = rospy.Subscriber(self.topics[index], rospy.AnyMsg,
                                                self.init_callback,
                                                callback_args=(index),
                                                queue_size=1)

    def init_callback(self, msg, args):
        index = args  # args[0]
        topic_type = msg._connection_header['type']
        self.topic_types[index] = topic_type
        self.subs[index].unregister()

        topic_class = get_message_class(topic_type)
        rospy.loginfo(f"{index} subscribing to {topic_type} {topic_class}")
        self.subs[index] = rospy.Subscriber(self.topics[index], topic_class,
                                            self.callback,
                                            callback_args=(index),
                                            queue_size=10)

    def callback(self, msg, args):
        index = args
        msg_str = f"{msg}"
        if len(msg_str) > self.max_len:
            msg_str = f"{msg_str[:self.max_len]}..."
        rospy.loginfo(f"{index} {self.topics[index]} {self.topic_types[index]}\n{msg_str}")


if __name__ == '__main__':
    rospy.init_node('multi_echo')
    print(sys.argv)
    node = MultiEcho()
    rospy.spin()
