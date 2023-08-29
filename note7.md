

## 路径规划

在ROS的导航功能包集 navigation 中提供了 `move_base 功能包`，用于实现此功能

> sudo apt install ros-<ROS版本>-navigation

`move_base 功能包`提供了基于动作(action)的路径规划实现，move_base 可以根据给定的目标点，控制机器人底盘运动至目标位置，并且在运动过程中会连续反馈机器人自身的姿态与目标点的状态信息。move_base 主要由`全局路径规划`与`本地路径规划`组成。

#### 1. 核心节点

move_base

#### 2. 订阅的 Action

- move_base/goal
    - move_base_msgs/MoveBaseActionGoal
    - move_base 的运动规划目标
- move_base/cancel
    - actionlib_msgs/GoalID
    - 取消目标


#### 3. 发布的 Action

- move_base/feedback
    - move_base_msgs/MoveBaseActionFeedback
    - 连续反馈的信息，包含机器人底盘坐标
- move_base/status
    - actionlib_msgs/GoalStatusArray
    - 发送到move_base的目标状态信息
- move_base/result
    - move_base_msgs/MoveBaseActionResult
    - 操作结果(此处为空)

#### 4. 订阅的Topic

- move_base_simple/goal
    - geometry_msgs/PoseStamped
    - 运动规划目标(与action相比，没有连续反馈，无法追踪机器人执行状态)

#### 5. 发布的Topic

- cmd_vel
    - geometry_msgs/Twist
    - 输出到机器人底盘的运动控制消息

#### 6. 服务

- ~make_plan
    - nav_msgs/GetPlan
    - 请求该服务，可以获取给定目标的规划路径，但是并不执行该路径规划。
- ~clear_unknown_space
    - std_srvs/Empty
    - 允许用户直接清除机器人周围的未知空间。
- ~clear_costmaps
    - std_srvs/Empty
    - 允许清除代价地图中的障碍物，可能会导致机器人与障碍物碰撞，请慎用。


## 代价地图

静态地图无法直接应用于导航，其基础之上需要添加一些辅助信息的地图，比如时时获取的障碍物数据，基于静态地图添加的`膨胀区`等数据。

代价地图有两张: `global_costmap`(全局代价地图) 和 `local_costmap`(本地代价地图)，前者用于全局路径规划，后者用于本地路径规划。

代价地图的组成(由以下任意排列组合组成)：
- Static Map Layer：静态地图层，SLAM构建的静态地图。
- Obstacle Map Layer：障碍地图层，传感器感知的障碍物信息。
- Inflation Layer：`膨胀层`，在以上两层地图上进行膨胀（向外扩张），以避免机器人的外壳会撞上障碍物。
- Other Layers：自定义costmap


## 碰撞算法




## move_base使用

## move_base 基础配置

> touch launch/move_base_config.launch

```xml
<launch>
    <!-- respawn 为 false，意味着该节点关闭后，不会被重启； -->
    <!-- clear_params 为 true，意味着每次启动该节点都要清空私有参数然后重新载入； -->
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
        <rosparam file="$(find hello_nav)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find hello_nav)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find hello_nav)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find hello_nav)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find hello_nav)/param/base_local_planner_params.yaml" command="load" />
    </node>

</launch>
```

> touch param/costmap_common_params.yaml param/local_costmap_params.yaml param/global_costmap_params.yaml param/base_local_planner_params.yaml

1. costmap_common_params.yaml 

通用参数：包括:机器人的尺寸、距离障碍物的安全距离、传感器信息等

```yaml
#机器人几何参，如果机器人是圆形，设置 robot_radius,如果是其他形状设置 footprint
robot_radius: 0.12 #圆形
# footprint: [[-0.12, -0.12], [-0.12, 0.12], [0.12, 0.12], [0.12, -0.12]] #其他形状

obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到距离小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代价地图中 3.5 米以外的障碍物

#膨胀半径，扩展在碰撞区域以外的代价区域，使得机器人规划路径避开障碍物
inflation_radius: 0.2
#代价比例系数，越大则代价值越小
cost_scaling_factor: 3.0

#地图类型
map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，你可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}

```

2. global_costmap_params.yaml

全局代价地图参数设置

```yaml
global_costmap:
  global_frame: map #地图坐标系
  robot_base_frame: base_footprint #机器人坐标系
  # 以此实现坐标变换
  update_frequency: 1.0 #代价地图更新频率
  publish_frequency: 1.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间
  static_map: true # 是否使用一个地图或者地图服务器来初始化全局代价地图，如果不使用静态地图，这个参数为false.
```


3. local_costmap_params.yaml

局部代价地图参数设置

```yaml
local_costmap:
  global_frame: odom #里程计坐标系
  robot_base_frame: base_footprint #机器人坐标系
  update_frequency: 10.0 #代价地图更新频率
  publish_frequency: 10.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间
  static_map: false  #不需要静态地图，可以提升导航效果
  rolling_window: true #是否使用动态窗口，默认为false，在静态的全局地图中，地图不会变化
  width: 3 # 局部地图宽度 单位是 m
  height: 3 # 局部地图高度 单位是 m
  resolution: 0.05 # 局部地图分辨率 单位是 m，一般与静态地图分辨率保持一致
```

4. base_local_planner_params.yaml

基本的局部规划器参数配置，这个配置文件设定了机器人的最大和最小速度限制值，也设定了加速度的阈值。

```yaml
TrajectoryPlannerROS:
  # Robot Configuration Parameters
  max_vel_x: 0.5 # X 方向最大速度
  min_vel_x: 0.1 # X 方向最小速速
  max_vel_theta:  1.0 # 
  min_vel_theta: -1.0
  min_in_place_vel_theta: 1.0
  acc_lim_x: 1.0 # X 加速限制
  acc_lim_y: 0.0 # Y 加速限制
  acc_lim_theta: 0.6 # 角速度加速限制
  # Goal Tolerance Parameters，目标公差
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05
  # Differential-drive robot configuration
  # 是否是全向移动机器人
  holonomic_robot: false
  # Forward Simulation Parameters，前进模拟参数
  sim_time: 0.8
  vx_samples: 18
  vtheta_samples: 20
  sim_granularity: 0.05
```


#### 参数配置技巧

全局路径规划与本地路径规划虽然设置的参数是一样的，但是二者路径规划和避障的职能不同，可以采用不同的参数设置策略:
- 全局代价地图可以将膨胀半径和障碍物系数设置的偏大一些
- 本地代价地图可以将膨胀半径和障碍物系数设置的偏小一些
这样，在全局路径规划时，规划的路径会尽量远离障碍物，而本地路径规划时，机器人即便偏离全局路径也会和障碍物之间保留更大的自由空间，从而避免了陷入“假死”的情形。


## move_base 集成


> touch launch/move_base_demo.launch

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find hello_nav)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find hello_nav)/launch/amcl_config.launch" />
    <!-- 运行move_base节点 -->
    <include file="$(find hello_nav)/launch/move_base_config.launch" />
    <!-- 运行rviz -->
    <!-- args="-d $(find hello_nav)/rviz/nav.rviz" -->
    <node pkg="rviz" type="rviz" name="rviz"  />
</launch>
```

启动

> roslaunch hello_nav move_base_demo.launch

在 rviz 中添加组件

- 类型为 Map 
    - 命名为静态地图
    - topic 设置为 /map
- 类型为 Map 
    - 命名为 全局代价地图
    - topic 设置为 /move_base/global_costmap/costmap
    - Color Scheme = costmap
- 类型为 Map 
    - 命名为 本地代价地图
    - topic 设置为 /move_base/local_costmap/costmap
- 类型为 PoseArray
    - topic 设置为 /particlecloud
- 类型为 LaserScan
    - 命名为 LaserScan
    - topic 设置为 /scan
    - size = 0.05
- 类型为 OdomMerty
    - topic 设置为 /odom
    - Coariance 取消勾选
- 类型为 Path
    - 命名为 Path_global
    - topic 设置为 /move_base/NavfnROS/plan
- 类型为 Path
    - 命名为 Path_local
    - topic 设置为 /move_base/TrajectoryPlannerROS/local_plan
    - 颜色=red

点击 2D Nav Goal 设置一个目标




## Action通信

在ROS中提供了actionlib功能包集，用于实现 action 通信

在请求和响应的过程中，服务端还可以`连续的反馈`当前任务进度，客户端可以接收连续反馈并且还可以取消任务

一般适用于耗时的请求响应场景，用以获取连续的状态反馈

#### 自定义action文件

导入依赖：actionlib actionlib_msgs

> mkdir action

> touch action/AddInts.action

```
#目标值
int32 num
---
#最终结果
int32 result
---
#连续反馈
float64 progress_bar
```

> vi CMakeLists.txt

```
find_package
(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)

add_action_files(
  FILES
  AddInts.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo04_action
 CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs
#  DEPENDS system_lib
)
```

重新编译

> catkin_make

编译后会生成一些中间文件(位于 /devel/lib/python2/dist-packages/hello_nav/msg/)


#### 编码实现

> touch scripts/action_server.py

```py
#! /usr/bin/env python
import rospy
import actionlib
from hello_nav.msg import *

class MyActionServer:
    def __init__(self):
        #SimpleActionServer(name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("addInts",AddIntsAction,self.cb,False)
        self.server.start()
        rospy.loginfo("服务端启动")


    def cb(self, goal):
        rospy.loginfo("服务端处理请求:")
        #1.解析目标值
        num = goal.num
        #2.循环累加，连续反馈
        rate = rospy.Rate(10)
        sum = 0
        for i in range(1,num + 1):
            # 累加
            sum = sum + i
            # 计算进度并连续反馈
            feedBack = i / num
            rospy.loginfo("当前进度:%.2f",feedBack)

            feedBack_obj = AddIntsFeedback()
            feedBack_obj.progress_bar = feedBack
            self.server.publish_feedback(feedBack_obj)
            rate.sleep()
        #3.响应最终结果
        result = AddIntsResult()
        result.result = sum        
        self.server.set_succeeded(result)
        rospy.loginfo("响应结果:%d",sum)
if __name__ == "__main__":
    rospy.init_node("action_server")
    server = MyActionServer()
    rospy.spin()

```

> touch scripts/action_client.py

```py
#! /usr/bin/env python

import rospy
import actionlib
from hello_nav.msg import *

def done_cb(state,result):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("响应结果:%d",result.result)

def active_cb():
    rospy.loginfo("服务被激活....")

def fb_cb(fb):
    rospy.loginfo("当前进度:%.2f",fb.progress_bar)

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("action_client")
    # 3.创建 action Client 对象
    client = actionlib.SimpleActionClient("addInts",AddIntsAction)
    # 4.等待服务
    client.wait_for_server()
    # 5.组织目标对象并发送
    goal_obj = AddIntsGoal()
    goal_obj.num = 10
    client.send_goal(goal_obj, done_cb, active_cb, fb_cb)
    # 6.编写回调, 激活、连续反馈、最终响应
    # 7.spin
    rospy.spin()

```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
catkin_install_python(PROGRAMS
  scripts/action_server.py
  scripts/action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

> . ./devel/setup.bash

> rosrun hello_nav action_server.py

> rosrun hello_nav action_client.py