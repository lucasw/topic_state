#!/usr/bin/env python
"""
Lucas Walter
Print rosout_agg to stdout/stderr just like live node/launch would
"""

import logging
import sys

import rospy
from rospy.impl.rosout import _rospy_to_logging_levels
from rosgraph_msgs.msg import Log
# TODO(lucasw) https://github.com/lucasw/ros_comm/blob/salsa_noetic_aggregated has format_msg()
from rosgraph.roslogging import (
    _color_reset,
    format_msg,
)


def log_callback(log):
    logger_levels_to_names = logging.getLevelNamesMapping()
    names_to_logger_level = {}
    for key, value in logger_levels_to_names.items():
        names_to_logger_level[value] = key
    logging_level = _rospy_to_logging_levels[log.level]
    logger_name = names_to_logger_level[logging_level]
    # print(f"log level {log.level} {logging_level} {logger_name}")
    msg, color = format_msg(log.msg, None, log.name, log.file,
                            log.line, log.function, logger_name, None,
                            log.header.stamp.to_sec())
    if color is None:
        color = ""
    text = color + msg + _color_reset

    if logging_level < logging.WARNING:
        print(text)
    else:
        print(text, file=sys.stderr)


def main():
    rospy.init_node("rosout_to_stdout")
    rospy.Subscriber("rosout_agg", Log, log_callback, queue_size=8)
    rospy.spin()


if __name__ == "__main__":
    main()
