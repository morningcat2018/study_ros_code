#! /usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped

topicName = "/tf"

if __name__ == "__main__":
    rospy.init_node("tf_staic_sub")
    rospy.loginfo("node:tf_staic_sub")
    # PointStamped
    ps = PointStamped()
    ps.header.frame_id = "laser"
    ps.header.stamp = rospy.Time.now()
    ps.point.x = 1.0
    ps.point.y = 3.0
    ps.point.z = 0.0
    # sub
    rate = rospy.Rate(3)
    buffer = tf2_ros.Buffer()
    sub = tf2_ros.TransformListener(buffer)
    while not rospy.is_shutdown():
        try:
            transform_result = buffer.transform(ps, "base_link")
            rospy.loginfo(
                "transform after result:(%.2f,%.2f,%.2f),cankaoxi:%s",
                transform_result.point.x,
                transform_result.point.y,
                transform_result.point.z,
                transform_result.header.frame_id,
            )
        except Exception as e:
            rospy.logwarn("error:%s", e)
        rate.sleep()
