import Adafruit_PCA9685
import smbus
import time
import sys
import numpy as np
import rospy
import std_msgs
import nav_msgs
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry, Path
import threading
import copy
import logging
logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

#  m1(cw)   m2(ccw)
#      -     -
#        - -
#        - -
#      -     -
#  m3(ccw)  m4(cw)
#
# Target inputs are in radians
# Motor inputs to PWM driver are in [0,1], later scaled to pulsewidth 1ms-2ms
# Gyro scale is maximal (+-2000 deg/s)
# Acc scale is ??

class AHRS:
    DEVICE_ADDR = 0x68  # MPU6050 device address
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47
    GYRO_SCALER = (2 * np.pi / 360. ) * (2000. / (2 ** 15)) # +- 2000 dps across a signed 16 bit value 
    ACC_SENSITIVITY = 16384.0

    def __init__(self):
        print("Initializing the MPU6050. ")

        self.bus = smbus.SMBus(1)

        # write to sample rate register
        self.bus.write_byte_data(AHRS.DEVICE_ADDR, AHRS.SMPLRT_DIV, 7)

        # Write to power management register
        self.bus.write_byte_data(AHRS.DEVICE_ADDR, AHRS.PWR_MGMT_1, 1)

        # Write to Configuration register
        self.bus.write_byte_data(AHRS.DEVICE_ADDR, AHRS.CONFIG, 0)

        # Write to Gyro configuration register
        self.bus.write_byte_data(AHRS.DEVICE_ADDR, AHRS.GYRO_CONFIG, 24)

        # Write to interrupt enable register
        self.bus.write_byte_data(AHRS.DEVICE_ADDR, AHRS.INT_ENABLE, 1)

        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0

        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.quat = [0, 0, 0, 1]

        self.gyro_integration_coeff = 0.95
        self.acc_integration_coeff = 1.0 - self.gyro_integration_coeff
        self.gyro_z_deadzone = 0.03

        self.timestamp = time.time()
        
        print("Finished initializing the MPU6050. ")

    def _read_raw_data(self, addr):
        # Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(AHRS.DEVICE_ADDR, addr)
        low = self.bus.read_byte_data(AHRS.DEVICE_ADDR, addr + 1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from mpu6050
        if (value > 32768):
            value = value - 65536
        return value

    def update(self):
        # Read Accelerometer raw value
        self.acc_x = self._read_raw_data(
            AHRS.ACCEL_XOUT_H) / AHRS.ACC_SENSITIVITY
        self.acc_y = self._read_raw_data(
            AHRS.ACCEL_YOUT_H) / AHRS.ACC_SENSITIVITY
        self.acc_z = self._read_raw_data(
            AHRS.ACCEL_ZOUT_H) / AHRS.ACC_SENSITIVITY

        # Read Gyroscope raw value
        self.gyro_x = self._read_raw_data(
            AHRS.GYRO_XOUT_H) * AHRS.GYRO_SCALER
        self.gyro_y = self._read_raw_data(
            AHRS.GYRO_YOUT_H) * AHRS.GYRO_SCALER
        self.gyro_z = self._read_raw_data(
            AHRS.GYRO_ZOUT_H) * AHRS.GYRO_SCALER

        # Calculate roll, pitch and yaw and corresponding quaternion
        t = time.time()
        dt = t - self.timestamp

        acc_x_dir = np.arctan2(self.acc_x, np.sqrt(
            self.acc_y ** 2 + self.acc_z ** 2))
        acc_y_dir = np.arctan2(self.acc_y, np.sqrt(
            self.acc_x ** 2 + self.acc_z ** 2))

        self.roll = self.gyro_integration_coeff * \
            (self.roll + self.gyro_x * dt) + \
            self.acc_integration_coeff * acc_y_dir
        self.pitch = self.gyro_integration_coeff * \
            (self.pitch - self.gyro_y * dt) + \
            self.acc_integration_coeff * acc_x_dir

        if abs(self.gyro_z) > self.gyro_z_deadzone:
            self.yaw = self.yaw + self.gyro_z * dt
        self.quat = self.e2q(self.roll, self.pitch, self.yaw)

        self.timestamp = t
        
        #print(acc_y_dir, self.gyro_x)
        #print(self.roll, self.pitch, self.yaw)
        #print(self.acc_x, self.acc_y, self.acc_z)

        print(self.gyro_x, self.gyro_y, self.gyro_z)

        return self.roll, self.pitch, self.yaw, self.quat, self.timestamp

    def e2q(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]


class PWMDriver:
    def __init__(self, motors_on):
        self.motors_on = motors_on

        if not self.motors_on:
            return

        self.pwm_freq = 50
        self.servo_ids = [0, 1, 2, 3]

        print("Initializing the PWMdriver. ")
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.pwm_freq)

        self.arm_escs()
        
        print("Finished initializing the PWMdriver. ")

    def write_servos(self, vals):
        '''

        :param vals: Throttle commmands [0,1] corresponding to min and max values
        :return: None
        '''

        if not self.motors_on:
            return

        for id in self.servo_ids:
            pulse_length = ((vals[id] + 1) * 1000) / ((1000000. / self.pwm_freq) / 4096.)
            self.pwm.set_pwm(id, 0, int(pulse_length))

    def arm_escs(self):
        print("Setting escs to lowest value. ")

        for id in self.servo_ids:
            self.write_servos([0, 0, 0, 0])

        time.sleep(2)


class ROSInterface:
    def __init__(self, update_rate=50):
        print("Initializing the ROS interface node")
        
        self.quad_pose_pub = rospy.Publisher(
            'quad_pose', PoseStamped, queue_size=10)
        rospy.Subscriber("quad_teleop", Joy,
                         self._ros_quad_teleop_callback, queue_size=10)

        self.quad_teleop_lock = threading.Lock()
        self.joy_input = None
        self.throttle = 0
        self.throttle_delta = 0.01
        self.target_yaw = 0
        self.target_yaw_delta = 0.03

        rospy.init_node('ros_quad_interface_node')
        self.ros_rate = rospy.Rate(update_rate)
                
        print("Finished initializing the ROS interface node")

    def _ros_quad_teleop_callback(self, data):
        with self.quad_teleop_lock:
            self.joy_input = data
    
    def read_control_inputs(self):
        if self.joy_input is None:
            return 0, 0, 0, 0
        else:
            with self.quad_teleop_lock:
                joy_message = copy.deepcopy(self.joy_input)

            yaw_inc, _, th_minus, roll, pitch, th_plus = joy_message.axes
            self.throttle = np.clip(self.throttle + (((th_plus + 1) / 2.) - ((th_minus + 1) / 2.)) * self.throttle_delta, 0, 1)
            
            if abs(yaw_inc) > 0.03:
                self.target_yaw -= yaw_inc * self.target_yaw_delta # Minus here because left stick is -1 and that has to correspond to positive yaw

            return roll, pitch, self.target_yaw, self.throttle

    def publish_telemetry(self, timestamp, quat):
        msg = PoseStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = "map"

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.quad_pose_pub.publish(msg)


class Controller:
    def __init__(self, motors_on=False):
        self.motors_on = motors_on

        print("Initializing the Controller, motors_on: {}".format(self.motors_on))

        self.AHRS = AHRS()
        self.PWMDriver = PWMDriver(self.motors_on)
        self.ROSInterface = ROSInterface(update_rate=50)

        self.ros_rate = self.ROSInterface.ros_rate

        self.p_roll = 0.7
        self.p_pitch = 0.7
        self.p_yaw = 0.3

        self.d_roll = 0.3
        self.d_pitch = 0.3
        self.d_yaw = 0.1

        self.e_roll_prev = 0
        self.e_pitch_prev = 0
        self.e_yaw_prev = 0
        
        print("Finished initializing the Controller")

    def loop_control(self):
        '''
        Target inputs are in radians, throttle is in [0,1]
        Roll, pitch and yaw are in radians. 
        Motor outputs are sent to the motor driver as [0,1]
        '''
        print("Starting the control loop")
        while not rospy.is_shutdown():
            # Read target control inputs
            t_roll, t_pitch, t_yaw, throttle = self.ROSInterface.read_control_inputs()

            # Update sensor data
            roll, pitch, yaw, quat, timestamp = self.AHRS.update()

            # Target errors
            e_roll = t_roll - roll
            e_pitch = t_pitch - pitch
            e_yaw = t_yaw - yaw

            # Desired correction action
            roll_act = e_roll * self.p_roll + (e_roll - self.e_roll_prev) * self.d_roll
            pitch_act = e_pitch * self.p_pitch + (e_pitch - self.e_pitch_prev) * self.d_pitch
            yaw_act = e_yaw * self.p_yaw + (e_yaw - self.e_yaw_prev) * self.d_yaw

            self.e_roll_prev = e_roll
            self.e_pitch_prev = e_pitch
            self.e_yaw_prev = e_yaw

            m_1_act_total = + roll_act / (2 * np.pi) + pitch_act / (2 * np.pi) + yaw_act / (2 * np.pi)
            m_2_act_total = - roll_act / (2 * np.pi) + pitch_act / (2 * np.pi) - yaw_act / (2 * np.pi)
            m_3_act_total = + roll_act / (2 * np.pi) - pitch_act / (2 * np.pi) - yaw_act / (2 * np.pi)
            m_4_act_total = - roll_act / (2 * np.pi) - pitch_act / (2 * np.pi) + yaw_act / (2 * np.pi)
            
            max_act = np.max([m_1_act_total, m_1_act_total, m_1_act_total, m_2_act_total])
            clipped_throttle = np.minimum(throttle, 1 - max_act)

            # Translate desired correction actions to servo commands
            m_1 = clipped_throttle + m_1_act_total 
            m_2 = clipped_throttle + m_2_act_total
            m_3 = clipped_throttle + m_3_act_total
            m_4 = clipped_throttle + m_4_act_total 

            if np.max([m_1, m_2, m_3, m_4]) > 1:
                print("Warning: motor commands exceed 1.0. This signifies an error in the system")
            
            # Write control to servos
            self.PWMDriver.write_servos([m_1, m_2, m_3, m_4])

            # Publish telemetry values
            self.ROSInterface.publish_telemetry(timestamp, quat)

            # Sleep to maintain correct FPS
            self.ros_rate.sleep()
            

if __name__ == "__main__":
    controller = Controller(motors_on=False)
    controller.loop_control()
    
    # TODO: Test the motors and input commands

