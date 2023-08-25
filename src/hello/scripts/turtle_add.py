#! /usr/bin/env python

import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

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
