#! /usr/bin/env python
# -*- coding:UTF-8 -*-

import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped


if __name__ == "__main__":
    rospy.init_node("tf_dynamic_sub")
    rospy.loginfo("node:tf_dynamic_sub")
    # 坐标系信息
    ps = PointStamped()
    ps.header.frame_id = "turtle1"
    ps.header.stamp = rospy.Time()
    ps.point.x = 1.0
    ps.point.y = 3.0
    ps.point.z = 0.0
    # 订阅信息
    rate = rospy.Rate(3)
    buffer = tf2_ros.Buffer()
    sub = tf2_ros.TransformListener(buffer)
    while not rospy.is_shutdown():
        try:
            transform_result = buffer.transform(ps, "world")
            rospy.loginfo(
                "转换后的坐标:(%.2f,%.2f,%.2f),参考系:%s",
                transform_result.point.x,
                transform_result.point.y,
                transform_result.point.z,
                transform_result.header.frame_id,
            )
        except Exception as e:
            rospy.logwarn("error:%s", e)
        rate.sleep()
