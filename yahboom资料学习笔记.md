
## 03、遥控体验教程 1.手机APP遥控教程

1. 手机下载APP

浏览器下载

2. 网络联入与小车相同的局域网里

3. 小车端执行命令

如果需要手动运动大程序，请先打开Ubuntu的终端，然后输入以下命令：

> python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py

## 05、ROSMASTER基础控制教程 1.更新扩展板固件

1. mcuisp（或flymcu）烧录软件

2. 扩展板单片机固件

最新版本的Rosmaster扩展板最新固件文件，名称为Rosmaster_XXX.hex

？未找到固件文件

3. 安装CH340驱动

？

## 05、ROSMASTER基础控制教程 2.关闭开机自启动大程序

打开Ubuntu系统的应用程序，搜索Startup Applications，将 start_rosmaster_app 前面的勾去掉

临时开启命令

> python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py


## 05、ROSMASTER基础控制教程 3.安装Rosmaster驱动库

出厂系统自带的驱动库存放路径：~/Software/py_install


## 05、ROSMASTER基础控制教程 8.控制机器人运动

没看懂

## 06、Linux操作系统 3.远程控制

用户名【jetson】，主机名【yahboom】

- ssh
- scp
- vnc viewer




## 06、Linux操作系统 4.多机通讯配置

要求：

- 所有的主控都处于同一个网络下
- 选一个作为主机，其他全都是从机
- 在每台设备上安装ssh和chrony包，用于实现同步

修改从机的.bashrc文件

> sudo apt-get install chrony openssh-server

> sudo vim ~/.bashrc

```
# add content
export ROS_MASTER_URI=http://masterIp:11311
```

> source ~/.bashrc


## 06、Linux操作系统 5.静态ip和热点模式

1. 设置静态ip

2. 热点模式是什么作用


## 06、Linux操作系统 6.绑定设备ID

需要复习！

## 06、Linux操作系统 7.网页实时监控



## 06、Linux操作系统 9.1.JetsonNano烧录镜像

没看懂


## 07、ROS基础课程



turtle_tf_broadcaster.py
```py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()#定义一个tf广播
    #广播world与输入命名的turtle之间的tf变换
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('turtle1_turtle2_tf_broadcaster')#初始化ros节点
    turtlename = rospy.get_param('~turtle') #从参数服务器中获取turtle的名字
    #订阅/pose话题数据，也就是turtle的位姿信息
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
```


## 09、机器人控制 3.机器人信息发布

> sudo vim .bashrc

export ROBOT_TYPE=X3


## 09、机器人控制 4.机器人键盘控制

> cd ~/yahboomcar_ws/src/yahboom_ctrl/scripts

机器人底盘启动
> roslaunch yahboomcar_bringup bringup.launch    

键盘控制节点
> roslaunch yahboomcar_ctrl yahboom_keyboard.launch      


## 10、深度相机 1.Astra相机使用



## 10、深度相机 5.Astra自动驾驶

## 10、深度相机 7.RTAB-Map建图导航

可用

## 10、深度相机 8.ORB_SLAM2基础

## 10、深度相机 9.ORB_SLAM2_Octomap



## 11、激光雷达

- 了解 三角测距法
- 了解 TOF飞行时间测距法

#### 1.雷达基础 EAI系列

1. 配置

> sudo vim ~/.bashrc

.bashrc 中如果没有下面这句话，就需要根据购买雷达型号手动添加，如果有这句话，直接修改雷达型号。例如：4ROS激光雷达
```sh
export RPLIDAR_TYPE=4ROS   # a1, a2, a3, s1, s2，4ROS，X3
```

> . ~/.bashrc

2. 使用雷达(4ROS雷达)

> roslaunch ydlidar_ros_driver TG.launch

通过终端打印话题数据来查看雷达是否启动正常

> rostopic echo /scan

RVIZ 查看扫描结果

> roslaunch ydlidar_ros_driver lidar_view.launch

3. 使用雷达(X3/X3Pro雷达)

> roslaunch ydlidar_ros_driver X2.launch

> rostopic echo /scan

> roslaunch ydlidar_ros_driver lidar_view.launch

4. lanuch 文件

~/software/library_ws/src/ydlidar_ros_driver-master/launch/X2.launch

```xml
<launch>
    <arg name="frame_id" default="laser"/>
  <node name="ydlidar_lidar_publisher"  pkg="ydlidar_ros_driver"  type="ydlidar_ros_driver_node" output="screen" respawn="false" >
    <!-- string property -->
    <param name="port"         type="string" value="/dev/rplidar"/>  
    <param name="frame_id" type="string" value="$(arg frame_id)"/>
    <param name="ignore_array"     type="string" value="-90,90"/>

    <!-- int property -->
    <param name="baudrate"         type="int" value="115200"/>  
    <!-- 0:TYPE_TOF, 1:TYPE_TRIANGLE, 2:TYPE_TOF_NET -->
    <param name="lidar_type"       type="int" value="1"/>  
    <!-- 0:YDLIDAR_TYPE_SERIAL, 1:YDLIDAR_TYPE_TCP -->
    <param name="device_type"         type="int" value="0"/>  
    <param name="sample_rate"         type="int" value="3"/>  
    <param name="abnormal_check_count"         type="int" value="4"/>  

    <!-- bool property -->
    <param name="resolution_fixed"    type="bool"   value="true"/>
    <param name="auto_reconnect"    type="bool"   value="true"/>
    <param name="reversion"    type="bool"   value="false"/>
    <param name="inverted"    type="bool"   value="true"/>
    <param name="isSingleChannel"    type="bool"   value="true"/>
    <param name="intensity"    type="bool"   value="false"/>
    <param name="support_motor_dtr"    type="bool"   value="true"/>
    <param name="invalid_range_is_inf"    type="bool"   value="true"/>
    <param name="point_cloud_preservative"    type="bool"   value="false"/>

    <!-- float property -->
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.1" />
    <param name="range_max"    type="double" value="12.0" />
    <!-- frequency is invalid, External PWM control speed -->
    <param name="frequency"    type="double" value="10.0"/>

  </node>
  <!--node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
    args="0.0 0.0 0.2 0.0 0.0 0.0 /base_footprint /laser_frame 40" /-->
</launch>
```

主要调试参数：
- angle_min 参数：雷达左侧角度
- angle_max 参数：雷达右侧角度

#### 1.雷达基础 思岚系列

运行rplidar节点，在rviz中查看

> roslaunch rplidar_ros view_rplidar.launch

运行 rplidar 节点并使用测试应用程序查看

> roslaunch rplidar_ros rplidar.launch     # 开启雷达

> rosrun rplidar_ros rplidarNodeClient     # 获取并打印雷达数据

建图测试

> roslaunch rplidar_ros test_gmapping.launch

> rosrun rqt_tf_tree rqt_tf_tree


~/software/library_ws/src/rplidar_ros/launch/rplidar.launch

```xml
<launch>
    <arg name="lidar_type" value="$(env RPLIDAR_TYPE)" doc="lidar_type type [a1,a2,a3,s1,s2]"/>
    <arg name="frame_id" default="laser"/>
    <arg name="shielding_angle" default="30"/>
    <!-- scan filtering node -->
    <node name="scan_filter" pkg="rplidar_ros" type="scan_filter.py" output="screen" respawn="true">
        <param name="shielding_angle" type="double" value="$(arg shielding_angle)"/>
    </node>
    <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode" output="screen" respawn="true">
        <param name="serial_port" type="string" value="/dev/rplidar"/>
        <param name="serial_baudrate" type="int" value="115200" if="$(eval arg('lidar_type') == 'a1')"/>
        <param name="serial_baudrate" type="int" value="115200" if="$(eval arg('lidar_type') == 'a2')"/>
        <param name="serial_baudrate" type="int" value="256000" if="$(eval arg('lidar_type') == 'a3')"/>
        <param name="serial_baudrate" type="int" value="256000" if="$(eval arg('lidar_type') == 's1')"/>
        <param name="serial_baudrate" type="int" value="1000000" if="$(eval arg('lidar_type') == 's2')"/>
        <param name="frame_id" type="string" value="$(arg frame_id)"/>
        <param name="inverted" type="bool" value="false"/>
        <param name="angle_compensate" type="bool" value="true"/>
        <param name="scan_mode" type="string" value="Sensitivity" if="$(eval arg('lidar_type') == 'a3')"/>
        <param name="scan_mode" type="string" value=" " unless="$(eval arg('lidar_type') == 'a3')"/>
        <remap from="scan" to="scan_raw"/>
    </node>
</launch>
```

- shielding_angle 参数：屏蔽雷达数据的角度，范围【0，360】，可根据实际情况调节
- gmapping只适用于 单帧二维激光点数小于1440的点，如果单帧激光点数大于1440，那么就会出现【[mapping-4] process has died】 这样的问题。所以在使用S2激光雷达时，需要对S2点的数量做了一个稀释。

如果不需要过滤雷达数据，把【rplidar.launch】文件中以下内容注释或者删除掉

```xml
    <!-- scan filtering node -->
    <node name="scan_filter" pkg="rplidar_ros" type="scan_filter.py" output="screen" respawn="true">
        <param name="shielding_angle" type="double" value="$(arg shielding_angle)"/>
    </node>
    <!-- 然后修改【rplidarNode】节点 -->
    <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode" output="screen" respawn="true">
        <!-- <remap from="scan" to="scan_raw"/>   删除 -->
        <remap from="scan" to="scan"/>      <!-- 增加 -->
    </node>
```

#### 2.雷达避障

一键启动（robot端），执行命令后，小车就开始运动

> roslaunch yahboomcar_laser laser_Avoidance.launch

动态调试参数

> rosrun rqt_reconfigure rqt_reconfigure

左侧选择 laser_Avoidance 栏

```
参数	范围	解析
【linear】	【0.0，1.0】	小车线速度
【angular】	【0.0，5.0】	小车角速度
【LaserAngle】	【10，90】	激光雷达检测角度（左右一侧角度）
【ResponseDist】	【0.0，8.0】	小车响应距离
【switch】	【False，True】	小车运动【开始/暂停】
```


#### 3.雷达警卫

一键启动，执行命令后，小车就开始运动

> roslaunch yahboomcar_laser laser_Warning.launch 

动态调试参数

> rosrun rqt_reconfigure rqt_reconfigure

左侧选择 laser_Warning 栏

```
【ang_Kp】、【ang_Ki】、【ang_Kd】：小车角速度PID调试
【LaserAngle】	【10，90】	激光雷达检测角度（左右一侧角度）
```

#### 4.雷达跟随

一键启动，执行命令后，小车就开始运动。

> roslaunch yahboomcar_laser laser_Tracker.launch

动态调试参数

> rosrun rqt_reconfigure rqt_reconfigure

[左侧选择 laser_Tracker 栏](https://www.yahboom.com/build.html?id=5329&cid=529)

#### 5.机器人巡逻

一键启动（robot端）

> roslaunch yahboomcar_bringup patrol.launch


#### 6.gmapping建图算法

- 优点：Gmapping可以实时构建室内地图，在构建小场景地图所需的计算量较小且精度较高
- 缺点：不适合构建大场景地图，没有回环检测
- 注意：建图时，速度慢一些效果越好（注要是旋转速度要慢些）

> sudo vim .bashrc

查找【ROBOT_TYPE】参数，修改相应的车型

```
export ROBOT_TYPE=X3   # ROBOT_TYPE: X1 X3 X3plus R2 X7
```

1. 启动命令（robot端）(从下列三种方式中选择一个)

> roslaunch yahboomcar_nav laser_bringup.launch           # laser + yahboomcar

> roslaunch yahboomcar_nav laser_usb_bringup.launch       # mono + laser + yahboomcar

> roslaunch yahboomcar_nav laser_astrapro_bringup.launch  # Astra + laser + yahboomcar

2. 建图命令（robot端）

> roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping

- 【use_rviz】参数：是否开启rviz可视化
- 【map_type】参数：设定建图算法【gmapping】

开启可视化界面（虚拟机端）

> roslaunch yahboomcar_nav view_map.launch

3. 控制机器人

键盘控制机器人移动

> rosrun teleop_twist_keyboard teleop_twist_keyboard.py    # 系统集成

> roslaunch yahboomcar_ctrl yahboom_keyboard.launch        # 自定义

手柄控制机器人移动

...

4. 地图保存

> rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/my_map    # 第一种方式

> bash ~/yahboomcar_ws/src/yahboomcar_nav/maps/map.sh                              # 第二种方式

地图将被保存到~/yahboomcar_ws/src/yahboomcar_nav/maps/文件夹下，一个pgm图片，一个yaml文件



#### 11.AMCL自适应蒙特卡洛定位





#### 12.导航避障

#### 13.APP建图与导航


#### 14、teb路径规划算法（R2车型专用）


## 12、多机编队	2.多机导航





## 13、深度学习	

1.KNN识别手写数字
2.TensorFlow基础使用
3.pytorch基础使用（jetson）
4.yolov5模型训练（jetson）
5.yolov5+tensorrt加速（jetson）
6.yolov4-tiny

看不懂

## 15/自动驾驶	

1、使用yolov5训练交通标志 
2、使用TensorRt加速识别标志 
3、标定自动驾驶数据 
4、采集数据 
5、训练模型 
6、模型转化与应用 
7、开启自动驾驶 



