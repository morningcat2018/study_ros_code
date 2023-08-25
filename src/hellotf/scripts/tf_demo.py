#! /usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf import transformations

topicName = "/tf_static"

if __name__ == "__main__":
    rospy.init_node("tf_staic_pub")
    rospy.loginfo("node:tf_staic_pub")
    # base
    base = TransformStamped()
    base.header.frame_id = "base_link"
    base.header.stamp = rospy.Time.now()
    base.child_frame_id = "laser"
    # translation
    base.transform.translation.x = 0.3
    base.transform.translation.y = 0.0
    base.transform.translation.z = 0.5
    # from euler to rotation
    (
        base.transform.rotation.x,
        base.transform.rotation.y,
        base.transform.rotation.z,
        base.transform.rotation.w,
    ) = transformations.quaternion_from_euler(0, 0, 0)
    # publiser
    pub = tf2_ros.StaticTransformBroadcaster()
    pub.sendTransform(base)
    rospy.spin()
