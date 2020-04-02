#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn

class RC2Twist():
    def __init__(self):
        rospy.init_node('rc2twist')
        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def normalize_rc(self, value):
        normalized_value = (value - 1520.0)/400.0
        return normalized_value
    
    def filter_deadzone(self, value, deadzone_range):
        value = 0.0 if abs(value) < deadzone_range else value

    def rc_callback(self, rc_callback_data):
        rc_rssi = rc_callback_data.rssi
        norm_value = [0.0, 0.0, 0.0, 0.0, 0.0]
        pub_twist_msg = Twist()
        
        for i in range(5):
            norm_value[i] = self.normalize_rc(rc_callback_data.channels[i])
            norm_value[i] = self.filter_deadzone(norm_value[i], 0.02)

        norm_linear_vel_x = norm_value[1]
        norm_linear_vel_y = norm_value[0]
        norm_angular_vel_z = norm_value[3]
        norm_channel = norm_value[2]
        norm_e_stop = norm_value[4]

        # Emergency Stop
        if norm_e_stop > 0.5:
            rospy.logwarn('EMERGENCY_STOP!')
            linear_vel_x = 0.0
            linear_vel_y = 0.0
            angular_vel_z = 0.0
        else:
            max_linear_vel_x = 1.0
            max_linear_vel_y = 1.0
            max_angular_vel_z = 0.7

            linear_vel_x = norm_linear_vel_x * max_linear_vel_x
            linear_vel_y = norm_linear_vel_y * max_linear_vel_y
            angular_vel_z = norm_angular_vel_z * max_angular_vel_z

            linear_vel_x = max(min(linear_vel_x, max_linear_vel_x), -max_linear_vel_x)
            linear_vel_y = max(min(linear_vel_y, max_linear_vel_y), -max_linear_vel_y)
            angular_vel_z = max(min(angular_vel_z, max_angular_vel_z), -max_angular_vel_z)
        
        pub_twist_msg.linear.x = linear_vel_x
        pub_twist_msg.linear.y = linear_vel_y
        pub_twist_msg.angular.z = angular_vel_z

        rospy.loginfo(rc_callback_data)
        rospy.loginfo(norm_value)
        rospy.loginfo('twist_msg:\n{}'.format(pub_twist_msg))
        self.twist_pub.publish(pub_twist_msg)
    
    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    try:
        rc2in = RC2Twist()
        rc2in.run() 
    except:
        pass