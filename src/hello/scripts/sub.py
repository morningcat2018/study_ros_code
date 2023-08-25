#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import String

topicName = "topic1"


def dealMsg(msg: String):
    rospy.loginfo("sub1: " + msg.data)


if __name__ == "__main__":
    rospy.init_node("sub1")
    rospy.loginfo("node:sub1")
    sub = rospy.Subscriber(topicName, String, dealMsg, queue_size=10)
    rospy.spin()
