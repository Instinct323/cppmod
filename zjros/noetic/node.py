#!/usr/bin/env python
# coding: utf-8
import sys

import rospy


class NodeBase:

    def __init__(self, name):
        rospy.init_node(name, sys.argv)
        # log methods
        self.info = rospy.loginfo
        self.warn = rospy.logwarn
        self.error = rospy.logerr
        self.fatal = rospy.logfatal

    def __enter__(self):
        self.info(f"Successful initialization.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


if __name__ == '__main__':
    with NodeBase("demo") as node:
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
