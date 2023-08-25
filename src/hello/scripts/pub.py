#! /usr/bin/env python

import rospy
from std_msgs.msg import String

topicName = "topic1"

if __name__ == "__main__":
    rospy.init_node("pub1")
    rospy.loginfo("node:pub1")
    pub = rospy.Publisher(topicName, String, queue_size=10)
    msg = String()
    rate = rospy.Rate(3)
    count = 0
    while not rospy.is_shutdown():
        count += 1
        msg.data = "my name is pub1 : count-" + str(count)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
