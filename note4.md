

## 1. 创建 ros 包

> catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs

## 2. ROS文件系统

```
WorkSpace --- 自定义的工作空间
    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。
    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件、动态&静态链接库、可执行文件等。
    |--- src: 源码
        |-- package：功能包(ROS基本单元)包含多个节点、库与配置文件，包名所有字母小写，只能由字母、数字与下划线组成
            |-- CMakeLists.txt 配置编译规则，比如源文件、依赖项、目标文件
            |-- package.xml 包信息，比如:包名、版本、作者、依赖项...(以前版本是 manifest.xml)
            |-- scripts 存储python文件
            |-- src 存储C++源文件
            |-- include 头文件
            |-- msg 消息通信格式文件
            |-- srv 服务通信格式文件
            |-- action 动作格式文件
            |-- launch 可一次性运行多个节点 
            |-- config 配置信息
        |-- CMakeLists.txt: 编译的基本配置
```

## 3. ROS文件系统相关命令

安装 ROS功能包

> apt install 包名

删除某个功能包

> apt purge 包名

列出所有功能包

> rospack list

查找某个功能包是否存在，如果存在返回安装路径

> rospack find 包名

运行指定的ROS节点

> rosrun 包名 xxx.py

执行某个包下的 launch 文件

> roslaunch 包名 xxx.launch

## 4. ROS计算图

> rqt_graph

## 5. 话题通信

```py
from xxx.msg import Person

# 发布方
pub = rospy.Publisher("/topic", Person, queue_size=10)
pub.publish(person1)

# 订阅方
def doPersonCallback(person1):
    person1.xxx
sub = rospy.Subscriber("/topic", Person, doPersonCallback, queue_size=10)
rospy.spin()
```

## 6. 服务通信

```py
from xxx.srv import AddInts,AddIntsRequest,AddIntsResponse

# 发布方
def doRequestCallback(request):
    # request.xxx
    resp = AddIntsResponse(yyy)
    return resp
server = rospy.Service("/service", AddInts, doRequestCallback)
rospy.spin()

# 订阅方
client = rospy.ServiceProxy("/service", AddInts)
client.wait_for_service()
req = AddIntsRequest()
resp = client.call(req)

```

## 7. rosnode rostopic rosmsg rosservice rossrv

```
rosnode ping xxx    测试到节点的连接状态
rosnode list    列出活动节点
rosnode info xxx   打印节点信息
rosnode machine    列出指定设备上节点
rosnode kill xxx   杀死某个节点
rosnode cleanup    清除不可连接的节点
```

```
rostopic bw     显示主题使用的带宽
rostopic delay  显示带有 header 的主题延迟
rostopic echo   打印消息到屏幕
rostopic find   根据类型查找主题
rostopic hz     显示主题的发布频率
rostopic info   显示主题相关信息
rostopic list   显示所有活动状态下的主题
rostopic pub    将数据发布到主题
rostopic type   打印主题类型
```

```
rosmsg show    显示消息描述
rosmsg info    显示消息信息
rosmsg list    列出所有消息
rosmsg md5    显示 md5 加密后的消息
rosmsg package    显示某个功能包下的所有消息
rosmsg packages    列出包含消息的功能包
```

```
rosservice args 打印服务参数
rosservice call    使用提供的参数调用服务
rosservice find    按照服务类型查找服务
rosservice info    打印有关服务的信息
rosservice list    列出所有活动的服务
rosservice type    打印服务类型
rosservice uri    打印服务的 ROSRPC uri
```

```
rossrv show    显示服务消息详情
rossrv info    显示服务消息相关信息
rossrv list    列出所有服务信息
rossrv md5    显示 md5 加密后的服务消息
rossrv package    显示某个包下所有服务消息
rossrv packages    显示包含服务消息的所有包
```

## 8. 时间

```py
now = rospy.Time.now()

time1 = rospy.Time(0)

du = rospy.Duration(3.3)
rospy.sleep(du) #休眠函数

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    rate.sleep() #休眠

rospy.Timer(rospy.Duration(1), doMsgCallback)
rospy.spin()
```

## 9. Python模块导入

```py
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/packageName/scripts")

```

## 10. launch文件

[ROS节点运行管理launch文件](http://www.autolabor.com.cn/book/ROSTutorials/5/45-rosjie-dianguan-li-launch-wen-jian.html)

## 11. 坐标msg消息

坐标转换实现中常用的 msg

- geometry_msgs/TransformStamped

```
std_msgs/Header header                      #头信息
  uint32 seq                                #|-- 序列号
  time stamp                                #|-- 时间戳
  string frame_id                           #|-- 坐标 ID
string child_frame_id                       #子坐标系的 id
geometry_msgs/Transform transform           #坐标信息
  geometry_msgs/Vector3 translation         #偏移量
    float64 x                               #|-- X 方向的偏移量
    float64 y                               #|-- Y 方向的偏移量
    float64 z                               #|-- Z 方向上的偏移量
  geometry_msgs/Quaternion rotation         #四元数
    float64 x                                
    float64 y                                
    float64 z                                
    float64 w

```


- geometry_msgs/PointStamped

```
std_msgs/Header header                      # 头
  uint32 seq                                # |-- 序号
  time stamp                                # |-- 时间戳
  string frame_id                           # |-- 所属坐标系的 id
geometry_msgs/Point point                   # 点坐标
  float64 x                                 # |-- x y z 坐标
  float64 y
  float64 z

```

## 12. 静态坐标变换

> catkin_create_pkg mcc_tf tf2 tf2_ros tf2_geometry_msgs roscpp rospy std_msgs geometry_msgs

```py
"""  
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 
    实现流程:
        3.创建 静态坐标广播器
        4.创建并组织被广播的消息
        5.广播器发送消息
        6.spin
"""
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    rospy.init_node("static_tf_pub")
    # 3.创建 静态坐标广播器
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # 4.创建并组织 被广播的消息
    tfs = TransformStamped()
    # --- 头信息
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    # --- 子坐标系
    tfs.child_frame_id = "radar"
    # --- 坐标系相对信息
    # ------ 偏移量
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5
    # ------ 四元数:从欧拉角转换到四元数
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    # 5.广播器发送消息
    broadcaster.sendTransform(tfs)
    # 6.spin
    rospy.spin()
```


```py
"""  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，
    转换成父级坐标系中的坐标点

    实现流程:
        3.创建 TF 订阅对象
        4.创建一个 radar 坐标系中的坐标点
        5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 world 的坐标
        6.spin
"""
import rospy
import tf2_ros
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
# from geometry_msgs.msg import PointStamped

if __name__ == "__main__":
    rospy.init_node("static_tf_sub")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():    
    # 4.创建一个 radar 坐标系中的坐标点
        point_source = PointStamped()
        point_source.header.frame_id = "radar"
        point_source.header.stamp = rospy.Time.now()
        point_source.point.x = 10
        point_source.point.y = 2
        point_source.point.z = 3
        try:
            # 5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 world 的坐标
            point_target = buffer.transform(point_source,"world")
            rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                            point_target.point.x,
                            point_target.point.y,
                            point_target.point.z)
        except Exception as e:
            rospy.logerr("异常:%s",e)
        # 6.spin
        rate.sleep()
```

linear(线速度) 下的xyz分别对应在x、y和z方向上的速度(单位是 m/s)

angular(角速度)下的xyz分别对应x轴上的翻滚、y轴上俯仰和z轴上偏航的速度(单位是rad/s)


## 12. 动态坐标变换


> catkin_create_pkg mcc_dynamic_tf tf2 tf2_ros tf2_geometry_msgs roscpp rospy std_msgs geometry_msgs

```py
"""  
    动态的坐标系相对姿态发布(一个坐标系相对于另一个坐标系的相对姿态是不断变动的)

    需求: 启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘
    控制乌龟运动，将两个坐标系的相对位置动态发布

    实现分析:
        1.乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
        2.订阅 turtle1/pose,可以获取乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
        3.将 pose 信息转换成 坐标系相对信息并发布
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.订阅 /turtle1/pose 话题消息
        4.回调函数处理
            4-1.创建 TF 广播器
            4-2.创建 广播的数据(通过 pose 设置)
            4-3.广播器发布数据
        5.spin
"""
import rospy
import tf2_ros
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

# 4.回调函数处理
def doPose(pose):
    # 4-1.创建 TF 广播器
    broadcaster = tf2_ros.TransformBroadcaster()
    #  4-2.创建 广播的数据(通过 pose 设置)
    tfs = TransformStamped()
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time(0)
    tfs.child_frame_id = "turtle1"
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0.0
    qtn = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
    (
        tfs.transform.rotation.x,
        tfs.transform.rotation.y,
        tfs.transform.rotation.z,
        tfs.transform.rotation.w,
    ) = qtn
    # 4-3.广播器发布数据
    broadcaster.sendTransform(tfs)

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("dynamic_tf_pub")
    # 3.订阅 /turtle1/pose 话题消息
    sub = rospy.Subscriber("/turtle1/pose", Pose, doPose)
    # 5.spin
    rospy.spin()
```

```py
"""  
    动态的坐标系相对姿态发布(一个坐标系相对于另一个坐标系的相对姿态是不断变动的)

    需求: 启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘
    控制乌龟运动，将两个坐标系的相对位置动态发布

    实现分析:
        1.乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
        2.订阅 turtle1/pose,可以获取乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
        3.将 pose 信息转换成 坐标系相对信息并发布
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.处理订阅的数据
"""
import rospy
import tf2_ros
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped

if __name__ == "__main__":
    rospy.init_node("dynamic_tf_sub")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():    
    # 4.创建一个 radar 坐标系中的坐标点
        point_source = PointStamped()
        point_source.header.frame_id = "turtle1"
        point_source.header.stamp = rospy.Time(0)
        point_source.point.x = 10
        point_source.point.y = 2
        point_source.point.z = 3
        try:
        # 5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 world 的坐标
            point_target = buffer.transform(point_source,"world",rospy.Duration(1))
            rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                            point_target.point.x,
                            point_target.point.y,
                            point_target.point.z)
        except Exception as e:
            rospy.logerr("异常:%s",e)
        # 6.spin
        rate.sleep()
```

## 13. 多坐标变换

发布方

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="son1" args="0.2 0.8 0.3 0 0 0 /world /son1" output="screen" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="son2" args="0.5 0 0 0 0 0 /world /son2" output="screen" />
</launch>
```



```py
"""  
    需求:
        现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，
        son1 相对于 world，以及 son2 相对于 world 的关系是已知的，
        求 son1 与 son2中的坐标关系，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标
    实现流程:   
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.调用 API 求出 son1 相对于 son2 的坐标关系
        5.创建一依赖于 son1 的坐标点，调用 API 求出该点在 son2 中的坐标
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped

if __name__ == "__main__":
    rospy.init_node("frames_sub_p")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            # 4.调用 API 求出 son1 相对于 son2 的坐标关系
            #lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            tfs = buffer.lookup_transform("son2", "son1", rospy.Time(0))
            rospy.loginfo("son1 与 son2 相对关系:")
            rospy.loginfo("父级坐标系:%s", tfs.header.frame_id)
            rospy.loginfo("子级坐标系:%s", tfs.child_frame_id)
            rospy.loginfo("相对坐标:x=%.2f, y=%.2f, z=%.2f",
                        tfs.transform.translation.x,
                        tfs.transform.translation.y,
                        tfs.transform.translation.z,
            )
            # 5.创建一个依赖于 son1 的坐标点，调用 API 求出该点在 son2 中的坐标
            point_source = PointStamped()
            point_source.header.frame_id = "son1"
            point_source.header.stamp = rospy.Time.now()
            point_source.point.x = 1
            point_source.point.y = 1
            point_source.point.z = 1
            point_target = buffer.transform(point_source, "son2", rospy.Duration(0.5))
            rospy.loginfo("point_target 所属的坐标系:%s", point_target.header.frame_id)
            rospy.loginfo("坐标点相对于 son2 的坐标:(%.2f,%.2f,%.2f)",
                        point_target.point.x,
                        point_target.point.y,
                        point_target.point.z
            )
        except Exception as e:
            rospy.logerr("错误提示:%s",e)
        rate.sleep()
```