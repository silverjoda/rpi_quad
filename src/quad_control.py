import Adafruit_PCA9685
import smbus
import time
import logging
import numpy as np
import rospy
import std_msgs, nav_msgs
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry, Path
import tf2_ros
import threading


#   m1(cw)  m2(ccw)
#      -     -
#        - -
#        - -
#      -     -
#  m3(ccw)  m4(cw)

class AHRS:

    DEVICE_ADDR = 0x68  # MPU6050 device address
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47
    GYRO_SENSITIVITY = 131.0
    ACC_SENSITIVITY = 16384.0


    def __init__(self):
        logging.info("Initializing the MPU6050. ")

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

        self.gyro_integration_coeff = 0.98
        self.acc_integration_coeff = 1.0 - self.gyro_integration_coeff

        self.timestamp = time.time()


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
        self.acc_x = self._read_raw_data(AHRS.ACCEL_XOUT_H) / AHRS.ACC_SENSITIVITY
        self.acc_y = self._read_raw_data(AHRS.ACCEL_YOUT_H) / AHRS.ACC_SENSITIVITY
        self.acc_z = self._read_raw_data(AHRS.ACCEL_ZOUT_H) / AHRS.ACC_SENSITIVITY

        # Read Gyroscope raw value
        self.gyro_x = self._read_raw_data(AHRS.GYRO_XOUT_H) / AHRS.GYRO_SENSITIVITY
        self.gyro_y = self._read_raw_data(AHRS.GYRO_YOUT_H) / AHRS.GYRO_SENSITIVITY
        self.gyro_z = self._read_raw_data(AHRS.GYRO_ZOUT_H) / AHRS.GYRO_SENSITIVITY

        # Calculate roll, pitch and yaw and corresponding quaternion
        t = time.time()
        dt = self.timestamp - t

        acc_x_dir = np.arctan2(self.acc_x, np.sqrt(self.acc_y ** 2 + self.acc_z))
        acc_y_dir = np.arctan2(self.acc_y, np.sqrt(self.acc_x ** 2 + self.acc_z))

        self.roll = self.gyro_integration_coeff * (self.roll + self.gyro_x * dt) + self.acc_integration_coeff * acc_x_dir
        self.pitch = self.gyro_integration_coeff * (self.pitch + self.gyro_y * dt) + self.acc_integration_coeff * acc_y_dir
        self.yaw = self.yaw + self.gyro_z * dt
        self.quat = self.e2q(self.roll, self.pitch, self.yaw)

        self.timestamp = t

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

        self.pwm_freq = 100
        self.servo_ids = [0, 1, 2, 3]

        logging.info("Initializing the PWMdriver. ")
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.pwm_freq)

        second_us = 1000000
        period_length_us = second_us // self.pwm_freq
        self.time_per_tick_us = period_length_us // (2 ** 10)

        self.arm_escs()


    def write_servos(self, vals):
        '''

        :param vals: Throttle commmands [0,1] corresponding to min and max values
        :return: None
        '''

        if not self.motors_on:
            return

        for id in self.servo_ids:
            length = (vals[id] * 1000 + 1000) // self.time_per_tick_us
            self.pwm.set_pwm(id, 0, length)


    def arm_escs(self):
        if not self.motors_on:
            return

        logging.info("Arming escs. ")

        for id in self.servo_ids:
            length = (2000) // self.time_per_tick_us
            self.pwm.set_pwm(id, 0, length)

        time.sleep(2)

        for id in self.servo_ids:
            length = (1000) // self.time_per_tick_us
            self.pwm.set_pwm(id, 0, length)

        time.sleep(2)



class Controller:
    def __init__(self, motors_on=False):
        self.motors_on = motors_on

        logging.info("Initializing the Controller, motors_on: {}".format(self.motors_on))

        self.AHRS = AHRS()
        self.PWMDriver = PWMDriver(self.motors_on)
        self.ROSInterface = ROSInterface(update_rate=50)

        self.ros_rate = self.ROSInterface.ros_rate

        self.p_roll = 0.1
        self.p_pitch = 0.1
        self.p_yaw = 0.1

        self.d_roll = 0.01
        self.d_pitch = 0.01
        self.d_yaw = 0.01

        self.e_roll_prev = 0
        self.e_pitch_prev = 0
        self.e_yaw_prev = 0


    def loop_control(self):

        # Read target control inputs
        t_roll, t_pitch, t_yaw, throttle = self.read_control_inputs()

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

        # Translate desired correction actions to servo commands
        m_1 = throttle + roll_act - pitch_act - yaw_act
        m_2 = throttle - roll_act - pitch_act + yaw_act
        m_3 = throttle + roll_act + pitch_act + yaw_act
        m_4 = throttle - roll_act + pitch_act - yaw_act

        # Write control to servos
        self.PWMDriver.write_servos([m_1, m_2, m_3, m_4])

        # Publish telemetry values
        self.ROSInterface.publish_telemetry()

        # Sleep to maintain correct FPS
        self.ros_rate.sleep()


    def read_control_inputs(self):
        joy_message = self.ROSInterface.joy_input
        if joy_message is None:
            return 0,0,0
        else:
            t_roll = joy_message.axes[0]
            t_pitch = joy_message.axes[1]
            t_yaw = joy_message.axes[2]
            throttle = joy_message.axes[3]
            return t_roll, t_pitch, t_yaw, throttle



class ROSInterface:
    def __init__(self, update_rate=50):
        self.ros_rate = rospy.Rate(update_rate)

        self.quad_pose_pub = rospy.Publisher('quad_pose', PoseStamped, queue_size=10)
        rospy.Subscriber("quad_teleop", Joy, self._ros_quad_teleop_callback, queue_size=10)

        self.quad_teleop_lock = threading.Lock()
        self.joy_input = None


    def _ros_quad_teleop_callback(self, data):
        with self.quad_teleop_lock:
            self.joy_input = data


    def publish_telemetry(self):
        pass


if __name__ == "__main__":
    controller = Controller(motors_on=False)
    #controller.loop_control()