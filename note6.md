
新建功能包 hello_nav

## SLAM建图

SLAM 算法有多种，这里使用 gmapping 进行学习

> sudo apt install ros-<ROS版本>-gmapping

gmapping 可以根据移动机器人`里程计数据`和`激光雷达数据`来绘制二维的`栅格地图`

对硬件的要求:
- 机器人可以发布里程计消息
- 机器人可以发布雷达消息

1. 核心节点

slam_gmapping

2. 订阅的 Topic

- tf
    - msg Type : tf/tfMessage
    - 用于雷达、底盘与里程计之间的坐标变换消息
- scan
    - msg Type : sensor_msgs/LaserScan
    - SLAM所需的雷达信息

3. 发布的Topic

- map_metadata
    - msg Type : nav_msgs/MapMetaData
    - 地图元数据，包括地图的宽度、高度、分辨率等，该消息会固定更新。
- map
    - msg Type : nav_msgs/OccupancyGrid
    - 地图栅格数据，一般会在rviz中以图形化的方式显示
- ~entropy
    - std_msgs/Float64


4. 发布的服务

- dynamic_map
    - nav_msgs/GetMap
    - 用于获取地图数据。

5. 参数

- base_frame(string, default:"base_link")
    - 机器人 基坐标系
- map_frame(string, default:"map")
    - 地图坐标系
- odom_frame(string, default:"odom")
    - 里程计坐标系

6. 所需的坐标变换

- 雷达坐标系 -> 基坐标系
    - 一般由 robot_state_publisher 或 static_transform_publisher 发布
- 基坐标系 -> 里程计坐标系
    - 一般由里程计节点发布

7. 发布的坐标变换

- 地图坐标系 -> 里程计坐标系
    - 地图到里程计坐标系之间的变换。

8. gmapping 配置文件

[参考文件](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/launch/slam_gmapping_pr2.launch)

slam_demo.launch

```xml
<launch>
    <param name="use_sim_time" value="true"/>

    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
      <remap from="scan" to="scan"/>
      <param name="base_frame" value="base_footprint"/><!--底盘坐标系-->
      <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="16.0"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>

    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />

    <node pkg="rviz" type="rviz" name="rviz" />
    <!-- 可以保存 rviz 配置并后期直接使用-->
    <!--
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_nav_sum)/rviz/gmapping.rviz"/>
    -->
</launch>

```

9. slam 建图

- 启动地图绘制的 launch 文件
    - roslaunch hello_nav slam_demo.launch
- 启动键盘键盘控制节点，用于控制机器人运动建图
    - rosrun teleop_twist_keyboard teleop_twist_keyboard.py
- 在 rviz 中添加组件，显示栅格地图
    - TF
        - Frame 下仅需要勾选 map/odom/base_link
    - Map
        - 设置话题 /map
    - LaserScan 雷达数据
        - 设置话题 /scan
        - Size = 0.05

在 rviz 里 Save Config As 将配置保存下来

## 地图服务

上一章中地图的保存是内存中的，我们需要将栅格地图序列化到的磁盘以持久化存储。
在ROS中，地图数据的序列化与反序列化可以通过 map_server 功能包实现。

> sudo apt install ros-<ROS版本>-map-server

map_server功能包中提供了两个节点: map_saver (将栅格地图保存到磁盘) 和 map_server (读取磁盘的栅格地图并以服务的方式提供出去)

1. map_saver 节点

订阅的 topic

- map  
    - nav_msgs/OccupancyGrid
    - 订阅此话题用于生成地图文件

地图保存launch文件: save_map.launch

```xml
<launch>
    <arg name="filename" value="$(find hello_nav)/map/nav" />
    <node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```
SLAM建图完毕后，执行该launch文件即可保存地图到磁盘 （共有 nav.pgm 与 nav.yaml 两个文件）

> roslaunch hello_nav save_map.launch

- nav.pgm
    - 本质是一张图片
- nav.yaml
    - 地图的元数据信息，用于描述图片

nav.yaml 解析

```
image: /home/workspace/demo1/src/hello/map/nav.pgm
resolution: 0.050000
origin: [-50.000000, -50.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

- image:被描述的图片资源路径，可以是绝对路径也可以是相对路径。（P304）
- resolution: 图片分片率(单位: m/像素)。
- origin: 地图中左下像素的二维姿势，为（x，y，偏航），偏航为逆时针旋转（偏航= 0表示无旋转）。
- occupied_thresh: 占用概率大于此阈值的像素被视为完全占用。
- free_thresh: 占用率小于此阈值的像素被视为完全空闲。
- negate: 是否应该颠倒白色/黑色自由/占用的语义。

2. map_server

发布的 topic

- map_metadata
    - nav_msgs / MapMetaData
    - 发布地图元数据
- map
    - nav_msgs / OccupancyGrid
    - 地图数据

提供的服务

- static_map
    - nav_msgs / GetMap
    - 通过此服务获取地图

参数

- frame_id（字符串，默认值：“map”）
    - 地图坐标系


通过 map_server 的 map_server 节点可以读取栅格地图数据： get_map.launch

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find hello_nav)/map/$(arg map)"/>
</launch>
```

执行该launch文件，该节点会发布话题:map(nav_msgs/OccupancyGrid)

> roslaunch hello_nav get_map.launch

在 rviz 中使用 map 组件可以显示栅格地图（添加 Map 组件，话题设置为 /map）

## 定位服务


- 定位: 推算机器人自身在全局地图中的位置
- SLAM中也包含定位算法实现，不过SLAM的定位是用于构建全局地图的，是属于导航开始之前的阶段
- 导航中，机器人需要按照设定的路线运动，通过定位可以判断机器人的实际轨迹是否符合预期。
    - 在ROS的导航功能包集navigation中提供了 `amcl 功能包`，用于实现导航中的机器人定位
    - sudo apt install ros-<ROS版本>-navigation
- AMCL(adaptive Monte Carlo Localization) 是用于2D移动机器人的`概率定位系统`，它实现了自适应（或KLD采样）`蒙特卡洛定位`方法，可以根据已有地图使用粒子滤波器推算机器人位置

1. 节点

amcl

2. 订阅的Topic

- scan
    - sensor_msgs/LaserScan
    - 激光雷达数据
- tf
    - tf/tfMessage
    - 坐标变换消息
- initialpose
    - geometry_msgs/PoseWithCovarianceStamped
    - 用来初始化粒子滤波器的均值和协方差
- map
    - nav_msgs/OccupancyGrid
    - 获取地图数据

3. 发布的Topic

- amcl_pose
    - geometry_msgs/PoseWithCovarianceStamped
    - 机器人在地图中的位姿估计
- particlecloud
    - geometry_msgs/PoseArray
    - 位姿估计集合，rviz中可以被 PoseArray 订阅然后图形化显示机器人的位姿估计集合
- tf
    - tf/tfMessage
    - 发布从 odom 到 map 的转换

4. 提供的服务


- global_localization
    - std_srvs/Empty
    - 初始化全局定位的服务
- request_nomotion_update
    - std_srvs/Empty
    - 手动执行更新和发布更新的粒子的服务
- set_map
    - nav_msgs/SetMap
    - 手动设置新地图和姿态的服务

5. 调用的服务

- static_map
    - nav_msgs/GetMap
    - 调用此服务获取地图数据

6. 参数

- ~odom_model_type(string, default:"diff")
    - 里程计模型选择(diff 差速、omni 全向轮)
- ~odom_frame_id(string, default:"odom")
    - 里程计坐标系
- ~base_frame_id(string, default:"base_link")
    - 机器人极坐标系
- ~global_frame_id(string, default:"map")
    - 地图坐标系

7. 坐标变换

amcl 通过估算机器人在地图坐标系下的姿态，再结合里程计提高定位准确度

- 里程计定位: 只是通过里程计数据实现 /odom_frame 与 /base_frame 之间的坐标变换
- amcl定位: 可以提供 /map_frame 、/odom_frame 与 /base_frame 之间的坐标变换



8. amcl使用

(1)编写amcl节点相关的launch文件

关于launch文件的实现，在amcl功能包下的example目录已经给出了示例，可以作为参考，具体实现:

> roscd amcl && ls examples

> touch launch/amcl_config.launch 

```xml
<launch>
  <node pkg="amcl" type="amcl" name="amcl">
    <!-- 里程计坐标系 -->
    <param name="odom_frame_id" value="odom"/>
    <!-- 添加机器人基坐标系 -->
    <param name="base_frame_id" value="base_footprint"/>
    <!-- 添加地图坐标系 -->
    <param name="global_frame_id" value="map"/>
    
    <!-- Publish scans from best pose at a max of 10 Hz -->
    <param name="odom_model_type" value="omni"/>
    <param name="odom_alpha5" value="0.1"/>
    <param name="transform_tolerance" value="0.2" />
    <param name="gui_publish_rate" value="10.0"/>
    <param name="laser_max_beams" value="30"/>
    <param name="min_particles" value="500"/>
    <param name="max_particles" value="5000"/>
    <param name="kld_err" value="0.05"/>
    <param name="kld_z" value="0.99"/>
    <param name="odom_alpha1" value="0.2"/>
    <param name="odom_alpha2" value="0.2"/>
    <!-- translation std dev, m -->
    <param name="odom_alpha3" value="0.8"/>
    <param name="odom_alpha4" value="0.2"/>
    <param name="laser_z_hit" value="0.5"/>
    <param name="laser_z_short" value="0.05"/>
    <param name="laser_z_max" value="0.05"/>
    <param name="laser_z_rand" value="0.5"/>
    <param name="laser_sigma_hit" value="0.2"/>
    <param name="laser_lambda_short" value="0.1"/>
    <param name="laser_lambda_short" value="0.1"/>
    <param name="laser_model_type" value="likelihood_field"/>
    <!-- <param name="laser_model_type" value="beam"/> -->
    <param name="laser_likelihood_max_dist" value="2.0"/>
    <param name="update_min_d" value="0.2"/>
    <param name="update_min_a" value="0.5"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="resample_interval" value="1"/>
    <param name="transform_tolerance" value="0.1"/>
    <param name="recovery_alpha_slow" value="0.0"/>
    <param name="recovery_alpha_fast" value="0.0"/>
  </node>
</launch>
```

(2)编写测试launch文件

amcl 节点是不可以单独运行的，运行 amcl 节点之前，需要先加载全局地图，然后启动 rviz 显示定位结果，上述节点可以集成进launch文件

> touch launch/amcl_demo.launch

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行rviz -->
    <!-- <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" /> -->
    <node pkg="rviz" type="rviz" name="rviz"/>
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find hello_nav)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find hello_nav)/launch/amcl_config.launch" />
</launch>
```

9. 执行

> roslaunch hello_nav amcl_demo.launch

在启动的 rviz 中，添加 posearray 插件，设置 topic 为 particlecloud 来显示 amcl 预估的当前机器人的位姿，箭头越是密集，说明当前机器人处于此位置的概率越高

