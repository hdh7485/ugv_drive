#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from dynamixel_workbench_msgs.srv import DynamixelCommand

class UGVDriver:
    def __init__(self):
        rospy.init_node('ugv_drive')
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback, queue_size=1)
        self.turn_motor = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)

    def twist_callback(self, twist_callback_data):
        # rospy.loginfo(twist_callback_data)
        steer_angle, steer_speed, wheel_speed = self.calculate_joint_kinetics(twist_callback_data)
        rospy.loginfo("steer[rad]:{}".format(steer_angle))
        # rospy.loginfo(steer_speed)
        rospy.loginfo("speed[rad/s]:{}".format(wheel_speed))

        result0 = self.turn_dynamixel(1, steer_angle[2]) 
        result1 = self.turn_dynamixel(2, steer_angle[1]) 
        result2 = self.turn_dynamixel(3, steer_angle[0]) 
        result3 = self.turn_dynamixel(4, steer_angle[3]) 

    def radian2value(self, radian):
        value = -501923 / 180 * 180 / math.pi * radian
        return value

    def normalize(self, radian):
        new_radian = radian
        while (new_radian <= -math.pi): new_radian += math.pi*2
        while (new_radian > math.pi): new_radian -= math.pi*2
        return newAngle

    def turn_dynamixel(self, motor_id, radian):
        try:
            value = self.radian2value(radian)
            resp1 = self.turn_motor('', motor_id, 'Goal_Position', value)
            return resp1.comm_result
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def calculate_joint_kinetics(self, twist_data):
        wheel_position = [{'x':0.6, 'y':0.4}, {'x':0.6, 'y':-0.4}, {'x':-0.6, 'y':-0.4}, {'x':-0.6, 'y':0.4}]
        wheel_radius = 0.0127
        steer_angle = [0.0, 0.0, 0.0, 0.0]
        steer_speed = [0.0, 0.0, 0.0, 0.0]
        wheel_speed = [0.0, 0.0, 0.0, 0.0]
        wheel_distance = [0.0, 0.0, 0.0, 0.0]
        wheel_angle_rad = [0.0, 0.0, 0.0, 0.0]
        target_steer_1 = [0.0, 0.0, 0.0, 0.0]
        target_steer_2 = [0.0, 0.0, 0.0, 0.0]
        target_speed_1 = [0.0, 0.0, 0.0, 0.0]
        target_speed_2 = [0.0, 0.0, 0.0, 0.0]

        x_vel = twist_data.linear.x
        y_vel = twist_data.linear.y
        angular_vel = twist_data.angular.z

        if abs(x_vel) < 0.001 and abs(y_vel) < 0.001 and abs(angular_vel) < 0.001:
            pass
        else:
            for i in range(4):
                wheel_distance[i] = math.sqrt(wheel_position[i]['x']**2 + wheel_position[i]['y']**2)
                # calculate direction of rotational vector
                #wheel_angle_rad[i] = atan4quad(m_vdExWheelYPosMM[i], m_vdExWheelXPosMM[i]);
                wheel_angle_rad[i] = math.atan2(wheel_position[i]['y'], wheel_position[i]['x'])

                # Rotational Portion
                local_x_vel = x_vel + angular_vel * wheel_distance[i] * -math.sin(wheel_angle_rad[i])
                local_y_vel = y_vel + angular_vel * wheel_distance[i] * math.cos(wheel_angle_rad[i])

                # calculate resulting steering angle
                # Wheel has to move in direction of resulting velocity vector of steering axis
                target_steer_1[i] = math.atan2(local_y_vel, local_x_vel)
                # calculate corresponding angle in opposite direction (+180 degree)
                target_steer_2[i] = target_steer_1[i] + math.pi

                # calculate absolute value of rotational rate of driving wheels in rad/s
                target_speed_1[i] = math.sqrt(local_x_vel**2 + local_y_vel**2) / wheel_radius
                # now adapt to direction (forward/backward) of wheel
                target_speed_2[i] = -target_speed_1[i]
            wheel_speed = target_speed_1
            steer_angle = target_steer_1
        return steer_angle, steer_speed, wheel_speed

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    try:
        ugv_driver = UGVDriver()
        ugv_driver.run()
    except:
        pass
