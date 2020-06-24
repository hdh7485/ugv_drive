#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList
import odrive
from odrive.enums import *
import time
from tf.transformations import quaternion_from_euler

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
        #rospy.loginfo(axis.motor)
        if axis.motor.error == 0:
            axis.motor.config.pre_calibrated = True
            self.odrive.save_configuration()
        else:
            rospy.logerr("{} state motor calibration failed".format(axis))

    def state_encoder_offset_calibration(self, axis):
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        #rospy.loginfo(axis.encoder)
        if axis.encoder.error == 0:
            axis.encoder.config.pre_calibrated = True
            self.odrive.save_configuration()
        else:
            rospy.logerr("{} state encoder offset calibration failed".format(axis))

    def rotate_rad_s_axis0(self, rad_per_sec):
        #count_per_sec = rad_per_sec * 2 * math.pi / 60 * 15
        #count_per_sec = rad_per_sec * 90 / (2*math.pi)
        count_per_sec = rad_per_sec * 45 / math.pi
        self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis0.controller.vel_setpoint = count_per_sec
        #rospy.loginfo(count_per_sec)
    
    def rotate_rad_s_axis1(self, rad_per_sec):
        #count_per_sec = rad_per_sec * 2 * math.pi / 60 * 15
        #count_per_sec = rad_per_sec * 90 / (2*math.pi)
        count_per_sec = rad_per_sec * 45 / math.pi
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.controller.vel_setpoint = count_per_sec

    def request_idle_state_axis0(self):
        self.axis0.requested_state = AXIS_STATE_IDLE
        
    def request_idle_state_axis1(self):
        self.axis1.requested_state = AXIS_STATE_IDLE

    def vel_estimate_axis0(self):
        return self.axis0.encoder.vel_estimate 

    def vel_estimate_axis1(self):
        return self.axis1.encoder.vel_estimate

class UGVDriver:
    def __init__(self):
        self.odom_timer = 0.1
        self.radius = 0.127 #m
        self.dt = self.odom_timer
        self.cpr = 90
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.current_steer_radian = [0, 0, 0, 0]  #radian
        self.current_wheel_speed = [0, 0, 0, 0]   #radian/sec

        rospy.init_node('ugv_drive')
        self.front_odrive = ODriveController(timeout=30, serial_number="20803592524B")
        self.back_odrive  = ODriveController(timeout=30, serial_number="20583592524B")
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback, queue_size=1)
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', 
                         DynamixelStateList, self.dynamixel_callback, queue_size=1)
        self.odom_publisher = rospy.Publisher('/ugv_odom', Odometry, queue_size=1)
        self.turn_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        rospy.Timer(rospy.Duration(self.odom_timer), self.odom_calculator)

    def odom_calculator(self, event=None):
        wheel_speed_1 = self.front_odrive.vel_estimate_axis1() * math.pi*2/self.cpr #FL rad/s
        wheel_speed_2 = self.front_odrive.vel_estimate_axis0() * math.pi*2/self.cpr #FR
        wheel_speed_3 = self.back_odrive.vel_estimate_axis0() * math.pi*2/self.cpr #BL
        wheel_speed_4 = self.back_odrive.vel_estimate_axis1() * math.pi*2/self.cpr #BR

        FL_steer = self.current_steer_radian[0]
        FR_steer = self.current_steer_radian[3]
        BL_steer = self.current_steer_radian[1]
        BR_steer = self.current_steer_radian[2]

        FL_speed = wheel_speed_1 * self.radius
        FR_speed = wheel_speed_2 * self.radius
        BL_speed = wheel_speed_3 * self.radius
        BR_speed = wheel_speed_4 * self.radius

        FL_speed_x = wheel_speed_1 * math.cos(FL_steer)
        FL_speed_y = wheel_speed_1 * math.sin(FL_steer)
        FR_speed_x = wheel_speed_2 * math.cos(FR_steer)
        FR_speed_y = wheel_speed_2 * math.sin(FR_steer)
        BL_speed_x = wheel_speed_3 * math.cos(BL_steer)
        BL_speed_y = wheel_speed_3 * math.sin(BL_steer)
        BR_speed_x = wheel_speed_4 * math.cos(BR_steer)
        BR_speed_y = wheel_speed_4 * math.sin(BR_steer)

        #rospy.loginfo("steer:{:4.2f} {:4.2f} {:4.2f} {:4.2f} 
        #        speed:{:4.2f} {:4.2f} {:4.2f} {:4.2f}".format(
        #        FL_steer, FR_steer, BL_steer, BR_steer, 
        #        FL_speed_y, FR_speed_y, BL_speed_y, BR_speed_y))

        x_mean = (FL_speed_x + FR_speed_x + BL_speed_x + BR_speed_x)/4
        y_mean = (FL_speed_y + FR_speed_y + BL_speed_y + BR_speed_y)/4
        #rospy.loginfo("x_maen:{:4.3f}, y_mean:{:4.3f}".format(x_mean, y_mean))

        #V = (FL_speed + FR_speed - BL_speed - BR_speed) / 4
        V = math.sqrt(x_mean**2 + y_mean**2)
        #rospy.loginfo("V:{:4.3}".format(V))

        Lx = 1.2
        Ly = 0.8
        beta = (FL_steer + FR_steer - BL_steer - BR_steer) / 4
        #rospy.loginfo("beta:{}".format(beta))
        yaw_rate = (2/Ly)*V*math.sin(beta)# + (2/Lx)*V*math.cos(beta)
        self.yaw = self.yaw + yaw_rate * self.dt
        Vx = V*math.cos(self.yaw + beta)
        rospy.loginfo("yaw:{}, Vx:{:4.3}".format(self.yaw, Vx))
        Vy = V*math.sin(self.yaw + beta)
        self.x = self.x + Vx * self.dt
        self.y = self.y + Vy * self.dt

        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom_msg = Odometry()
        odom_msg.header.seq = 0
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = Vx
        odom_msg.twist.twist.linear.y = Vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = yaw_rate
        self.odom_publisher.publish(odom_msg)

    def twist_callback(self, twist_callback_data):
        # rospy.loginfo(twist_callback_data)
        steer_angle, steer_speed, wheel_speed = self.calculate_joint_kinetics(twist_callback_data)
        result0 = self.turn_dynamixel(1, steer_angle[2]) # FL
        result1 = self.turn_dynamixel(2, steer_angle[1]) # BL
        result2 = self.turn_dynamixel(3, steer_angle[0]) # BR
        result3 = self.turn_dynamixel(4, steer_angle[3]) # FR
        self.front_odrive.rotate_rad_s_axis1(wheel_speed[2]) # FL
        self.back_odrive.rotate_rad_s_axis0(wheel_speed[1]) # BL
        self.back_odrive.rotate_rad_s_axis1(wheel_speed[0]) # BR
        self.front_odrive.rotate_rad_s_axis0(wheel_speed[3]) # FR

    def normalize_2pi(self, radian):
        normalized_radian = (radian + 2 * math.pi) % (2 * math.pi)
        return normalized_radian

    def normalize_pi(self, radian):
        new_radian = radian
        while (new_radian <= -math.pi): new_radian += math.pi*2
        while (new_radian > math.pi): new_radian -= math.pi*2
        return new_radian

    def dynamixel_callback(self, dynamixel_data):
        for motor in dynamixel_data.dynamixel_state:
            #rospy.loginfo(int(motor.id)-1)
            self.current_steer_radian[int(motor.id)-1] = self.value2radian(motor.present_position)
        #rospy.loginfo("steer_radian:{}".format(self.current_steer_radian))

    def value2radian(self, value):
        radian = float(value) / -501923.0 * math.pi
        return radian

    def radian2value(self, radian):
        value = int(-501923.0 / math.pi * radian)
        if value == -501923: value = 0
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
        wheel_position = [{'x':0.6, 'y':0.4}, {'x':0.6, 'y':-0.4}, {'x':-0.6, 'y':-0.4}, {'x':-0.6, 'y':0.4}] #m
        wheel_radius = 0.127 #m
        steer_angle = [0.0, 0.0, 0.0, 0.0] #rad
        steer_speed = [0.0, 0.0, 0.0, 0.0]
        wheel_speed = [0.0, 0.0, 0.0, 0.0] #rad/s
        wheel_distance = [0.0, 0.0, 0.0, 0.0]
        wheel_angle_rad = [0.0, 0.0, 0.0, 0.0]
        target_steer_1 = [0.0, 0.0, 0.0, 0.0]
        target_steer_2 = [0.0, 0.0, 0.0, 0.0]
        target_speed_1 = [0.0, 0.0, 0.0, 0.0] #rad/s
        target_speed_2 = [0.0, 0.0, 0.0, 0.0]

        x_vel = twist_data.linear.x #m/s
        y_vel = twist_data.linear.y
        angular_vel = twist_data.angular.z #rad/s

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
            #rospy.loginfo(wheel_speed)
            steer_angle = target_steer_1
        #rospy.loginfo(abs(steer_angle[2] - self.current_steer_radian[0]))
        if abs(steer_angle[2] - self.current_steer_radian[0]) > math.pi/2+0.02:
            steer_angle[2] += math.pi
            normalized_value = self.normalize_pi(steer_angle[2])
            if normalized_value > -math.pi and  normalized_value < math.pi:
                steer_angle[2] = normalized_value
                wheel_speed[2] *= -1
        if abs(steer_angle[1] - self.current_steer_radian[1]) > math.pi/2+0.02:
            inverse_value = steer_angle[1] + math.pi
            normalized_value = self.normalize_pi(inverse_value)
            if normalized_value > -math.pi and  normalized_value < math.pi:
                steer_angle[1] = normalized_value
                wheel_speed[1] *= -1
        if abs(steer_angle[0] - self.current_steer_radian[2]) > math.pi/2+0.02:
            steer_angle[0] += math.pi
            normalized_value = self.normalize_pi(steer_angle[0])
            if normalized_value > -math.pi and  normalized_value < math.pi:
                steer_angle[0] = normalized_value
                wheel_speed[0] *= -1
        if abs(steer_angle[3] - self.current_steer_radian[3]) > math.pi/2+0.02:
            steer_angle[3] += math.pi
            normalized_value = self.normalize_pi(steer_angle[3])
            if normalized_value > -math.pi and  normalized_value < math.pi:
                steer_angle[3] = normalized_value
                wheel_speed[3] *= -1
        #rospy.loginfo(wheel_speed)
        #rospy.loginfo(target_speed_1)

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
