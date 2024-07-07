#!/usr/bin/env python
import sys

import rospy
from rospy.msg import AnyMsg


rospy.init_node("connection_header")
msg = rospy.wait_for_message(sys.argv[1], AnyMsg)
print(dir(msg))
print(msg._connection_header)
