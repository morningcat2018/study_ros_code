#! /usr/bin/env python

import rospy
import os
import sys

path = os.path.abspath(".")
sys.path.insert(0, path + "/devel/lib/python2.7/dist-packages")
from hello.msg import Person

topicName = "topic2"

if __name__ == "__main__":
    rospy.init_node("pub_msg1")
    rospy.loginfo("node:pub_msg1")
    pub = rospy.Publisher(topicName, Person, queue_size=10)
    msg = Person()
    rate = rospy.Rate(3)
    count = 0
    while not rospy.is_shutdown():
        count += 1
        msg.name = "Jone-" + str(count)
        msg.age = 18
        msg.height = 1.67
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
