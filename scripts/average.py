#!/usr/bin/env python
# Copyright (c) 2019 Lucas Walter
# subscribe to a Float32 message (later support other number types and fields)
# and publish the average and standard deviation given a window size (in number of message or later time)

import math
import numpy as np
import rospy
import rostopic

# from roslib.message import get_message_class
# from rospy.msg import AnyMsg
from std_msgs.msg import Float32


class TopicState(object):
    def __init__(self):
        # TODO(lucasw) support many topics in param list
        # for now just one

        self.values = None
        self.window = rospy.get_param("~window", 200)
        rospy.loginfo('window {}'.format(self.window))
        self.update_period = rospy.get_param("~update_period", 0.1)

        self.float_pub = None
        self.float = 0.0
        self.float_sub = rospy.Subscriber('topic', Float32, self.callback, queue_size=10)
        self.avg_pub = rospy.Publisher('topic_average', Float32, queue_size=10)
        self.dev_pub = rospy.Publisher('topic_stddev', Float32, queue_size=10)
        self.update_timer = rospy.Timer(rospy.Duration(self.update_period), self.update)

    def callback(self, msg):
        # TODO(lucasw) if there is a header use that timestamp optionally
        if self.values is None:
            self.values = np.array(msg.data)
            return
        else:
            self.values = np.append(self.values, msg.data)

        if self.values.shape[0] > self.window:
            self.values = self.values[1:]

    def update(self, event):
        if self.values is None:
            return
        if self.values.shape[0] < 2:
            return

        average = np.average(self.values)
        self.avg_pub.publish(Float32(average))
        stddev = np.std(self.values)
        self.dev_pub.publish(Float32(stddev))
        # print('{} {}'.format(average, stddev))

if __name__ == '__main__':
    rospy.init_node('topic_state')
    topic_state = TopicState()
    rospy.spin()
