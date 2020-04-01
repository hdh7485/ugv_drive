#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn

def normalize_rc(value):
    normalized_value = (value - 1500)/500
    return normalized_value

def rc_callback(rc_callback_data):
    rc_rssi = rc_callback_data.rssi
    norm_values = [0, 0, 0, 0, 0]

    for i in range(5):
        norm_value[i] = normalize_rc(rc_callback_data.channels[i])

    norm_linear_vel_x = norm_value[1]
    norm_linear_vel_y = norm_value[0]
    norm_angular_vel_z = norm_value[3]
    norm_channel_1 = norm_value[2]
    norm_e_stop = norm_value[4]

    max_linear_vel_x = 1.0
    max_linear_vel_y = 1.0
    max_angular_vel_z = 0.7

    linear_vel_x = norm_linear_vel_x * max_linear_vel_x
    linear_vel_y = norm_linear_vel_y * max_linear_vel_y
    angular_vel_z = norm_angular_vel_z * max_angular_vel_z

    pub_twist_msg = Twist()

rospy.init_node('rc2twist')
rospy.Subscriber('/mavros/rc/in', RCIn, rc_callback)
rospy.spin()
