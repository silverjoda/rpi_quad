# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

motor_id = 2
pwm_freq = 100

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(pwm_freq)

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= pwm_freq      # 60 Hz
    pulse_length //= 4096     # 12 bits of resolution
    pulse //= pulse_length
    pwm.set_pwm(channel, motor_id, pulse)

set_servo_pulse(motor_id, 1900)
time.sleep(1)
set_servo_pulse(motor_id, 1100)
time.sleep(3)

print('Moving motor on channel 0, press Ctrl-C to quit...')
while True:
    # Move servo on channel O between extremes.
    set_servo_pulse(motor_id, 1100)
    time.sleep(2)
    set_servo_pulse(motor_id, 1300)
    time.sleep(2)
