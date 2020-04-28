#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList
import odrive
from odrive.enums import *
import time

class ODriveController:
    def __init__(self, timeout=30, serial_number=None):
        if serial_number == None:
            rospy.loginfo("Finding Odrive")
            self.odrive = odrive.find_any(timeout=timeout)
            rospy.loginfo("Odrive connected")
        else:
            rospy.loginfo("Finding {}".format(serial_number))
            self.odrive = odrive.find_any(timeout=timeout, serial_number=serial_number)
            rospy.loginfo("{} Odrive connected".format(serial_number))
        self.axis0 = self.odrive.axis0
        self.axis1 = self.odrive.axis1

        #self.axis0_init_params()
        #self.axis1_init_params()

        #self.state_motor_calibration(self.axis0)
        #self.state_encoder_offset_calibration(self.axis0)

        #self.state_motor_calibration(self.axis1)
        #self.state_encoder_offset_calibration(self.axis1)

        self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    def axis0_init_params(self):
        self.axis0.motor.config.pole_pairs = 15
        self.axis0.motor.config.resistance_calib_max_voltage = 4
        self.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
        self.axis0.motor.config.current_control_bandwidth = 100
        self.axis0.encoder.config.mode = ENCODER_MODE_HALL
        self.axis0.encoder.config.cpr = 90
        self.axis0.encoder.config.bandwidth = 100
        self.axis0.controller.config.pos_gain = 1
        self.axis0.controller.config.vel_gain = 0.02
        self.axis0.controller.config.vel_integrator_gain = 0.1
        self.axis0.controller.config.vel_limit = 1000
        self.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrive.save_configuration()

    def axis1_init_params(self):
        self.axis1.motor.config.pole_pairs = 15
        self.axis1.motor.config.resistance_calib_max_voltage = 4
        self.axis1.motor.config.requested_current_range = 25 #Requires config save and reboot
        self.axis1.motor.config.current_control_bandwidth = 100
        self.axis1.encoder.config.mode = ENCODER_MODE_HALL
        self.axis1.encoder.config.cpr = 90
        self.axis1.encoder.config.bandwidth = 100
        self.axis1.controller.config.pos_gain = 1
        self.axis1.controller.config.vel_gain = 0.02
        self.axis1.controller.config.vel_integrator_gain = 0.1
        self.axis1.controller.config.vel_limit = 1000
        self.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrive.save_configuration()

    def state_motor_calibration(self, axis):
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        rospy.loginfo(axis.motor)
        if axis.motor.error == 0:
            axis.motor.config.pre_calibrated = True
            self.odrive.save_configuration()
        else:
            rospy.logerr("{} state motor calibration failed".format(axis))

    def state_encoder_offset_calibration(self, axis):
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        rospy.loginfo(axis.encoder)
        if axis.encoder.error == 0:
            axis.encoder.config.pre_calibrated = True
            self.odrive.save_configuration()
        else:
            rospy.logerr("{} state encoder offset calibration failed".format(axis))

    def rotate_rpm_axis0(self, rad_per_sec):
        count_per_sec = rad_per_sec * 2 * math.pi / 60 * 15
        self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis0.controller.vel_setpoint = count_per_sec
    
    def rotate_rpm_axis1(self, rad_per_sec):
        count_per_sec = rad_per_sec * 2 * math.pi / 60 * 15
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.controller.vel_setpoint = count_per_sec

    def request_idle_state_axis0(self):
        self.axis0.requested_state = AXIS_STATE_IDLE
        
    def request_idle_state_axis1(self):
        self.axis1.requested_state = AXIS_STATE_IDLE

class UGVDriver:
    def __init__(self):
        rospy.init_node('ugv_drive')
        self.front_odrive = ODriveController(timeout=30, serial_number="20803592524B")
        self.back_odrive  = ODriveController(timeout=30, serial_number="20583592524B")
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback, queue_size=1)
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.dynamixel_callback, queue_size=1)
        self.turn_motor = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.current_steer_radian = [0, 0, 0, 0]

    def twist_callback(self, twist_callback_data):
        # rospy.loginfo(twist_callback_data)
        steer_angle, steer_speed, wheel_speed = self.calculate_joint_kinetics(twist_callback_data)
        # rospy.loginfo("steer[rad]:{}".format(steer_angle[2]))
        # rospy.loginfo("current_steer[rad]:{}".format(self.current_steer_radian[0]))
        # rospy.loginfo(steer_speed)
        # rospy.loginfo("speed[rad/s]:{}".format(wheel_speed))

        #if abs(steer_angle[2] - self.current_steer_radian[0]) > math.pi/2:
        #    steer_angle[2] += math.pi
        #    steer_angle[2] = self.normalize_2pi(steer_angle[2])
        #    wheel_speed[3] *= -1
        #if abs(steer_angle[1] - self.current_steer_radian[1]) > math.pi/2:
        #    steer_angle[1] += math.pi
        #    steer_angle[1] = self.normalize_2pi(steer_angle[1])
        #    wheel_speed[2] *= -1
        #if abs(steer_angle[0] - self.current_steer_radian[2]) > math.pi/2:
        #    steer_angle[0] += math.pi
        #    steer_angle[0] = self.normalize_2pi(steer_angle[0])
        #    wheel_speed[1] *= -1
        #if abs(steer_angle[3] - self.current_steer_radian[3]) > math.pi/2:
        #    steer_angle[3] += math.pi
        #    steer_angle[3] = self.normalize_2pi(steer_angle[3])
        #    wheel_speed[0] *= -1
        result0 = self.turn_dynamixel(1, steer_angle[2]) 
        result1 = self.turn_dynamixel(2, steer_angle[1]) 
        result2 = self.turn_dynamixel(3, steer_angle[0]) 
        result3 = self.turn_dynamixel(4, steer_angle[3]) 
        self.front_odrive.rotate_rpm_axis0(wheel_speed[3])
        self.front_odrive.rotate_rpm_axis1(wheel_speed[2])
        self.back_odrive.rotate_rpm_axis0(wheel_speed[1])
        self.back_odrive.rotate_rpm_axis1(wheel_speed[0])

    def normalize_2pi(self, radian):
        normalized_radian = (radian + 2 * math.pi) % (2 * math.pi)
        return normalized_radian

    def dynamixel_callback(self, dynamixel_data):
        for motor in dynamixel_data.dynamixel_state:
            #rospy.loginfo(int(motor.id)-1)
            self.current_steer_radian[int(motor.id)-1] = self.value2radian(motor.present_position)
        #rospy.loginfo(self.current_steer_radian)

    def value2radian(self, value):
        radian = float(value) / -501923.0 * math.pi
        return radian

    def radian2value(self, radian):
        value = -501923.0 / math.pi * radian
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

        if abs(steer_angle[2] - self.current_steer_radian[0]) > math.pi/2+0.02:
            steer_angle[2] += math.pi
            steer_angle[2] = self.normalize_2pi(steer_angle[2])
            wheel_speed[3] *= -1
        if abs(steer_angle[1] - self.current_steer_radian[1]) > math.pi/2+0.02:
            steer_angle[1] += math.pi
            steer_angle[1] = self.normalize_2pi(steer_angle[1])
            wheel_speed[2] *= -1
        if abs(steer_angle[0] - self.current_steer_radian[2]) > math.pi/2+0.02:
            steer_angle[0] += math.pi
            steer_angle[0] = self.normalize_2pi(steer_angle[0])
            wheel_speed[1] *= -1
        if abs(steer_angle[3] - self.current_steer_radian[3]) > math.pi/2+0.02:
            steer_angle[3] += math.pi
            steer_angle[3] = self.normalize_2pi(steer_angle[3])
            wheel_speed[0] *= -1
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
