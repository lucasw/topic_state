#!/usr/bin/env python
import sys

import rospy
from rospy.msg import AnyMsg


rospy.init_node("connection_header")
topic = sys.argv[1]
print(topic)
msg = rospy.wait_for_message(topic, AnyMsg)
# print(dir(msg))
print(f"type: {msg._type}")
print("connection header:")
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
for key, value in msg._connection_header.items():
    print(f"{key}:\n{value}")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
print()
