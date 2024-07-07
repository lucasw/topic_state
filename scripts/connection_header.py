#!/usr/bin/env python

import rospy
from rospy.msg import AnyMsg


rospy.init_node("connection_header")
msg = rospy.wait_for_message("topic", AnyMsg)
print(dir(msg))
print(msg._connection_header)
