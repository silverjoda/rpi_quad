# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import numpy as np

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
pwm_freq = 50 

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(pwm_freq)

def set_servo_cmd(channel, val):
    '''
    Command from [0,1]
    '''
    val_clip = np.clip(val, 0, 1)
    pulse_length = ((val_clip + 1) * 1000) / ((1000000. / pwm_freq) / 4096.)
    print(int(pulse_length))
    pwm.set_pwm(channel, 0, int(pulse_length))
    
set_servo_cmd(2, 0)
time.sleep(2)

print('Moving motor on channel 0, press Ctrl-C to quit...')
while True:
    for i in range(100):
        set_servo_cmd(2, i / 100.)
        time.sleep(0.2)
    set_servo_cmd(2, 0)
    time.sleep(3)
