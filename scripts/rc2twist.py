#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn

def rc_callback(rc_callback_data):
    pass

rospy.init_node('rc2twist')
rospy.Subscriber('/mavros/rc/in', RCIn, rc_callback)
rospy.spin()