#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

topicName = "/turtle1/cmd_vel"

if __name__ == "__main__":
    rospy.init_node("turtle")
    rospy.loginfo("node:turtle")
    #
    pub = rospy.Publisher(topicName, Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 1.25
    msg.angular.z = 0.5
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
