#!/usr/bin/env python
# Copyright (c) 2022 Lucas Walter
#
# subscribe to multiple messages and echo them all, interleave their output
# TODO(lucasw) allow other than 2 topics
# Also add exact and approximate time sync options

import sys
from threading import Lock

import message_filters
import rospy
from roslib.message import get_message_class


class MultiEcho(object):
    def __init__(self, topics):
        # TODO(lucasw) support many topics in param list
        # for now just one

        self.lock = Lock()

        self.max_len = rospy.get_param("~max_len", 250)
        self.use_sync = rospy.get_param("~use_sync", True)

        self.sync_sub = None
        self.topics = {}
        self.topic_types = {}
        self.classes = {}
        self.subs = {}
        for index in range(len(topics)):
            self.topics[index] = topics[index]
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
        rospy.loginfo(f"{index} found class for '{topic_type}': {topic_class}")
        if self.use_sync:
            self.classes[index] = topic_class
        else:
            self.subs[index] = rospy.Subscriber(self.topics[index], topic_class,
                                                self.callback,
                                                callback_args=(index),
                                                queue_size=10)

        with self.lock:
            if self.sync_sub is None and len(self.classes.keys()) == len(self.subs.keys()):
                rospy.loginfo(f"now setting up sync subscriber {self.topics.values()}")
                subs = {}
                for key in self.classes.keys():
                    subs[key] = message_filters.Subscriber(self.topics[key], self.classes[key])

                self.sync_sub = message_filters.TimeSynchronizer(subs.values(), queue_size=100)
                self.sync_sub.registerCallback(self.sync_callback)

    def callback(self, msg, args):
        index = args
        msg_str = f"{msg}"
        if len(msg_str) > self.max_len:
            msg_str = f"{msg_str[:self.max_len]}..."
        rospy.loginfo(f"{index} {self.topics[index]} {self.topic_types[index]}\n{msg_str}")

    def sync_callback(self, *msgs):
        text = f"\n\n----------------------------- sync callback ({len(msgs)}) -----------------------"
        stamp = rospy.Time.now()
        text += f"\nage {(stamp - msgs[0].header.stamp).to_sec():0.3f}s"
        rospy.loginfo(text)
        for ind, msg in enumerate(msgs):
            self.callback(msg, ind)


if __name__ == '__main__':
    rospy.init_node('multi_echo')
    node = MultiEcho(sys.argv[1:])
    rospy.spin()
