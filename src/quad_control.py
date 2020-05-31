import Adafruit_PCA9685
import smbus
import time
import logging
import numpy as np


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
    def __init__(self):
        pass
