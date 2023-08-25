#! /usr/bin/env python
# -*- coding:UTF-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf import transformations
from turtlesim.msg import Pose

topicName = "/turtle1/pose"


def doPose(pose):
    # 将位姿信息 转换成 坐标系信息
    ts = TransformStamped()
    ts.header.frame_id = "world"
    ts.header.stamp = rospy.Time()
    ts.child_frame_id = "turtle1"
    # 坐标系信息->偏移量
    ts.transform.translation.x = pose.x
    ts.transform.translation.y = pose.y
    ts.transform.translation.z = 0
    # 坐标系信息->四元数
    # from 欧拉角 to 四元数
    (
        ts.transform.rotation.x,
        ts.transform.rotation.y,
        ts.transform.rotation.z,
        ts.transform.rotation.w,
    ) = transformations.quaternion_from_euler(0, 0, pose.theta)

    # 发布数据
    pub = tf2_ros.TransformBroadcaster()
    pub.sendTransform(ts)


if __name__ == "__main__":
    rospy.init_node("tf_dynamic_pub")
    rospy.loginfo("node:tf_dynamic_pub")
    # 订阅小乌龟的位姿信息
    rospy.Subscriber(topicName, Pose, doPose, queue_size=10)
    rospy.spin()
