

## TF 坐标静态转换（重要）

1. 示例

> roslaunch turtle_tf2 turtle_tf2_demo.launch

2. 静态转换的发布和订阅

> cd src

> catkin_create_pkg hellotf roscpp rospy std_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs 

> cd .. && catkin_make

> cd src/hellotf

> mkdir scripts launch

> touch scripts/tf_pub.py


```py
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

```

> touch scripts/tf_sub.py

```py
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

```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
# update content

catkin_install_python(PROGRAMS
  scripts/tf_pub.py
  scripts/tf_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## TF 坐标动态转换（重要）

> touch scripts/tf_dynamic_pub.py

```py
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

```

> touch scripts/tf_dynamic_sub.py

```py
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

```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
# update content

catkin_install_python(PROGRAMS
  scripts/tf_pub.py
  scripts/tf_sub.py
  scripts/tf_dynamic_pub.py
  scripts/tf_dynamic_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## 多坐标转换（重要）



## 小乌龟跟随（练习）



## 坐标系关系查看

1. 坐标信息打印

2. rviz

3. tf2_tools

> rospack find tf2_tools


## rosbag

> rosbag record /turtle1/cmd_vel -o bags/hello.bag

> rosbag info bags/helloxxx

> rosbag play bags/helloxxx


## rqt 工具箱（常用）

> rosrun rqt_gui rqt_gui 

1. topic 

2. service

3. rqt_graph

Plugins -> Introspection -> Node Graph

4. rqt_console 日志筛选


## 代码

- [https://github.com/morningcat2018/study_ros_code](https://github.com/morningcat2018/study_ros_code)