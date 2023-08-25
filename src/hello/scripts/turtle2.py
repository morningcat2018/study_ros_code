#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

topicName = "/turtle1/cmd_vel"
topicName2 = "/turtle1/pose"


def getMessage(pose1: Pose):
    rospy.loginfo(
        "info: x(%.2f),y(%.2f),theta(%.2f),%.2f,%.2f",
        pose1.x,
        pose1.y,
        pose1.theta,
        pose1.linear_velocity,
        pose1.angular_velocity,
    )


if __name__ == "__main__":
    rospy.init_node("turtle2")
    rospy.loginfo("node:turtle2")
    #
    rospy.Subscriber(topicName2, Pose, getMessage, queue_size=5)
    rospy.spin()
