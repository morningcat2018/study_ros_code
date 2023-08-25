

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

> touch scripts/tf_sub.py

## TF 坐标动态转换（重要）


## 多坐标转换（重要）





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

