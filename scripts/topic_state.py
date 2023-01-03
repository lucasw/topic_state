#!/usr/bin/env python
# Copyright (c) 2019 Lucas Walter
# Subscribe to a list of any type of message, then publish a pulse/toggle for every
# received message.  This allows easy visualization of the flow of messages
# on the system when view in rqt_plot (or plotjuggler) similar to viewing
# the messages in rqt_bag.
# Furthermore, the output is published as a JointState which can be hooked
# up to an abstract RobotModel and then the reception of messages can be visualized
# in rviz.

import rospy

from rospy.msg import AnyMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class TopicState(object):
    def __init__(self):
        # TODO(lucasw) support many topics in param list
        # for now just one

        self.use_float = rospy.get_param("~use_float", True)
        self.float = None
        self.float_pub = None
        if self.use_float:
            self.float = 0.0
            self.float_pub = rospy.Publisher('topic_state_float', Float32, queue_size=10)

        self.use_joint_state = rospy.get_param("~use_joint_state", False)
        self.js = None
        self.js_pub = None

        if self.use_joint_state:
            self.js = JointState()
            self.js.name.append('topic')
            self.js.position.append(0.0)
            self.js_pub = rospy.Publisher('topic_state', JointState, queue_size=10)

        # self.label = rospy.get_param("~label", "topic")
        self.pulse_period = rospy.get_param("~period", 0.02)
        self.edge_time = rospy.get_param("~edge_time", 0.002)
        self.end_pulse_timer = None
        self.sub = rospy.Subscriber('topic', AnyMsg, self.callback, queue_size=4)

    def callback(self, msg):
        # TODO(lucasw) if there is a header use that timestamp optionally
        self.start_pulse()

    def start_pulse(self, event=None):
        if self.end_pulse_timer is not None:
            self.end_pulse_timer.shutdown()
        self.pub(0.0)
        rospy.sleep(rospy.Duration(self.edge_time))
        self.pub(1.0)
        self.end_pulse_timer = rospy.Timer(rospy.Duration(self.pulse_period),
                                           self.end_pulse,
                                           oneshot=True)

    def end_pulse(self, event=None):
        self.pub(1.0)
        rospy.sleep(rospy.Duration(self.edge_time))
        self.pub(0.0)
        self.end_pulse_timer = None

    def pub(self, value=0.0, stamp=None):
        if self.use_joint_state:
            self.pub_js(value, stamp)
        if self.use_float:
            self.pub_float(value)

    def pub_js(self, value=0.0, stamp=None):
        self.js.header.stamp = stamp
        if stamp is None:
            self.js.header.stamp = rospy.Time.now()  # or event.cur_real
        self.js.position[0] = value
        self.js_pub.publish(self.js)

    def pub_float(self, value=0.0, stamp=None):
        self.float = value
        self.float_pub.publish(self.float)


if __name__ == '__main__':
    rospy.init_node('topic_state')
    topic_state = TopicState()
    rospy.spin()
