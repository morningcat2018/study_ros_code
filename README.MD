

## base 

1. create project

> mkdir -p demo/src

> cd demo

> catkin_make

> tree .

```
├── build
├── devel
│   ├── setup.bash
│   ├── setup.sh
└── src
    └── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.c
```


2. create package

> cd src

> catkin_create_pkg hello roscpp rospy std_msgs

> tree .

```
.
├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
└── hello
    ├── CMakeLists.txt
    ├── include
    │   └── hello
    ├── package.xml
    └── src
```

> cd hello

> mkdir scripts msg srv action cfg launch

3. quickstart python

> touch scripts/pub.py

> vi scripts/pub.py

```python
#! /usr/bin/env python

import rospy
from std_msgs.msg import String

topicName = "topic1"

if __name__ == "__main__":
    rospy.init_node("pub1")
    rospy.loginfo("node:pub1")
    pub = rospy.Publisher(topicName, String, queue_size=10)
    msg = String()
    rate = rospy.Rate(3)
    count = 0
    while not rospy.is_shutdown():
        count+=1
        msg.data = "my name is pub1 : count-" + str(count)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
# add content

catkin_install_python(PROGRAMS
  scripts/pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

> cd ../..

rebuild

> catkin_make

strat ROS

> roscore

> source ./devel/setup.bash

> rosrun hello pub.py 

4. Subscriber topic

> cd src/hello

> touch scripts/sub.py

> vi scripts/sub.py

```python
#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import String

topicName = "topic1"

def dealMsg(msg):
	rospy.loginfo("sub1: " + msg.data)

if __name__ == "__main__":
	rospy.init_node("sub1")
	rospy.loginfo("node:sub1")
	sub = rospy.Subscriber(topicName, String, dealMsg, queue_size=10)
	rospy.spin()

```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
# update content

catkin_install_python(PROGRAMS
  scripts/pub.py
  scripts/sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

> rosrun hello sub.py 

5. customize msg

> touch msg/Person.msg

> vi msg/Person.msg

```
string name
int32 age
float32 height
```

> vi package.xml

```
  <!--add content-->
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

> vi CMakeLists.txt

```
# update content

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Person.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

> cd ../..

> catkin_make

> tree . > tree.txt

```
.
├── build
├── devel
│   ├── include
│   │   └── hello
│   │       └── Person.h
│   ├── lib
│   │   ├── pkgconfig
│   │   │   └── hello.pc
│   │   └── python2.7
│   │       └── dist-packages
│   │           └── hello
│   │               ├── __init__.py
│   │               └── msg
│   │                   ├── __init__.py
│   │                   └── _Person.py
│   ├── setup.bash
│   ├── setup.sh
├── src
│   ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
│   └── hello
│       ├── action
│       ├── cfg
│       ├── CMakeLists.txt
│       ├── include
│       │   └── hello
│       ├── launch
│       ├── msg
│       │   └── Person.msg
│       ├── package.xml
│       ├── scripts
│       │   ├── pub.py
│       │   └── sub.py
│       ├── src
│       └── srv
└── tree.txt
```

use customize msg

> cd src/hello/

> touch scripts/pub_msg.py

> vi scripts/pub_msg.py

```python
#! /usr/bin/env python

import rospy
import os
import sys
path = os.path.abspath(".")
sys.path.insert(0, path + "/devel/lib/python2.7/dist-packages")
from hello.msg import Person

topicName = "topic2"

if __name__ == "__main__":
    rospy.init_node("pub_msg1")
    rospy.loginfo("node:pub_msg1")
    pub = rospy.Publisher(topicName, Person, queue_size=10)
    msg = Person()
    rate = rospy.Rate(3)
    count = 0
    while not rospy.is_shutdown():
        count+=1
        msg.name = "Jone-" + str(count)
        msg.age = 18
        msg.height = 1.67
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
```


> touch scripts/sub_msg.py

> vi scripts/sub_msg.py

```python
#! /usr/bin/env python

import rospy
import os
import sys
path = os.path.abspath(".")
sys.path.insert(0, path + "/devel/lib/python2.7/dist-packages")
from hello.msg import Person

topicName = "topic2"

def dealMsg(person):
	rospy.loginfo("person info: name(%s),age(%d),height(%f)" , person.name, person.age, person.height)

if __name__ == "__main__":
	rospy.init_node("sub_msg1")
	rospy.loginfo("node:sub_msg1")
	sub = rospy.Subscriber(topicName, Person, dealMsg, queue_size=10)
	rospy.spin()
```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
# update content

catkin_install_python(PROGRAMS
  scripts/pub.py
  scripts/sub.py
  scripts/pub_msg.py
  scripts/sub_msg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## use srv 

> touch scripts/turtle_add.py

> vi scripts/turtle_add.py


```python
#! /usr/bin/env python

import rospy
from turtlesim.srv import Spawn,SpawnRequest,SpawnResponse

topicName = "/spawn"

if __name__ == "__main__":
    rospy.init_node("turtle_add")
    rospy.loginfo("node:turtle_add")
    # 
    client = rospy.ServiceProxy(topicName, Spawn)
    req = SpawnRequest()
    req.name = "py1"
    req.theta = 3.14
    req.x = 8
    req.y = 8
    client.wait_for_service()
    try:
        res = client.call(req)
        rospy.loginfo("new turtle : %s\n", res.name)
    except Exception as e:
        rospy.logerr(e)

```

> chmod +x scripts/*.py

> vi CMakeLists.txt

```
# update content

catkin_install_python(PROGRAMS
  scripts/pub.py
  scripts/sub.py
  scripts/pub_msg.py
  scripts/sub_msg.py
  scripts/turtle_add.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

> rosrun turtlesim turtlesim_node

> rosrun hello turtle_add.py

## launch

> touch scripts/turtlee.py

```python
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

topicName = "/turtle1/cmd_vel"

if __name__ == "__main__":

    rospy.init_node("turtle")
    rospy.loginfo("node:turtle")
    # 
    pub = rospy.Publisher(topicName, Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 1.25
    msg.angular.z = 0.5
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


```

skip one content

> touch launch/hello.launch

> vi launch/hello.launch

```xml
<launch>
    <!-- rosrun turtlesim turtlesim_node __ns:=space1 -->
    <!-- rosrun turtlesim turtlesim_node __name:=other_name -->
    <node pkg="turtlesim" type="turtlesim_node" name="spawn1" output="screen"/>
    <!-- rosrun hello turtlee.py -->
    <node pkg="hello" type="turtlee.py" name="my_plan" output="screen"/>
    <!-- rosrun teleop_twist_keyboard teleop_twist_keyboard.py  -->
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="twist_keyboard" output="screen">
        <remap from="/cmd_vel" to="/turtle1/cmd_vel"/>
    </node>
    <group ns="my_group1">
        <node pkg="turtlesim" type="turtlesim_node" name="spawn1" output="screen"/>
    </group>
</launch>
<!-- roslaunch hello xxx.launch -->
```

> roslaunch hello hello.launch

## command

1. rosnode

> rosnode list

> rosnode info /spawn1

2. rostopic

> rostopic list

> rostopic info /turtle1/cmd_vel

> rosmsg info geometry_msgs/Twist

3. rosservice

> rosservice list

> rosservice info /spawn

> rossrv info turtlesim/Spawn

> rosservice call /spawn "x: 5.0
y: 5.0
theta: 1.57
name: 'turtle2'" 


## doc

- [doc](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/13-rosji-cheng-kai-fa-huan-jing-da-jian/131-helloworldshi-xian-jian-jie.html)