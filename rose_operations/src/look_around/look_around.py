#! /usr/bin/env python

import rospy

from rose_gaze_controller.msg import *
from std_msgs.msg import *
import random
import actionlib

#import pdb; pdb.set_trace()
rospy.init_node("look_around")

lookat = actionlib.SimpleActionClient("/gaze_controller", LookAtAction)
tilt = rospy.Publisher("/neck_tilt_controller/command", Float64)
pan = rospy.Publisher("/neck_pan_controller/command", Float64)

def random_look():
    x = random.uniform(1,10)
    y = random.uniform(-2,2)
    z = random.uniform(0,2)
    
    g = LookAtGoal()
    g.target_point.header.frame_id="/base_link"
    g.target_point.point.x = x
    g.target_point.point.y = y
    g.target_point.point.z = z
    
    print g.target_point 
    lookat.send_goal(g)

def random_angles():
    t = random.uniform(-1, 0.4)
    p = random.uniform(-1, 1)

    print t,p
    tilt.publish(t)
    pan.publish(p)

go = True
while not rospy.is_shutdown():
    try:
        random_angles()
        rospy.sleep(5)
    except KeyboardInterrupt:
        print "Stopping"
        go = False
        exit(1)


