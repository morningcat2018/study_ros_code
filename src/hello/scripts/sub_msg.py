#! /usr/bin/env python

import rospy
import os
import sys

path = os.path.abspath(".")
sys.path.insert(0, path + "/devel/lib/python2.7/dist-packages")
from hello.msg import Person

topicName = "topic2"


def dealMsg(person):
    rospy.loginfo(
        "person info: name(%s),age(%d),height(%f)",
        person.name,
        person.age,
        person.height,
    )


if __name__ == "__main__":
    rospy.init_node("sub_msg1")
    rospy.loginfo("node:sub_msg1")
    sub = rospy.Subscriber(topicName, Person, dealMsg, queue_size=10)
    rospy.spin()
