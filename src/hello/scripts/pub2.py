#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import String

topicName = "topic1"

if __name__ == "__main__":
    rospy.init_node("pub2")
    rospy.loginfo("node:pub2")
    pub = rospy.Publisher(topicName, String, queue_size=10)
    msg = String()
    count = 0
    while not rospy.is_shutdown():
        count = count + 1
        msg.data = "my name is pub2: count-" + str(count)
        rospy.loginfo(msg)
        pub.publish(msg)
        time.sleep(1)
