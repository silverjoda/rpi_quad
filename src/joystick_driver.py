import time
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry, Path
import threading
import copy
import logging
logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
import pygame

class JoyRosQuadController():
    def __init__(self, update_rate=50):
        print("Initializing quadcopter ros joystick controller")
        
        self.joystick_control_pub = rospy.Publisher('quad_teleop', Joy, queue_size=10)
        rospy.init_node('ros_quad_joystick_controller_node')
        self.ros_rate = rospy.Rate(update_rate)

        pygame.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(joystick.get_name())
        time.sleep(0.5)

        while(True):
            print([joystick.get_axis(i) for i in range(joystick.get_numaxes())])
            print([joystick.get_button(i) for i in range(joystick.get_numbuttons())])
            print([joystick.get_hat(i) for i in range(joystick.get_numhats())])
            time.sleep(0.01)

        exit()
                
        print("Finished initializing the quadcopter ros joystick controller")
    
    def read_control_inputs(self):
        if self.joy_input is None:
            return 0, 0, 0, 0
        else:
            with self.quad_teleop_lock:
                joy_message = copy.deepcopy(self.joy_input)
            t_roll = joy_message.axes[0]
            t_pitch = joy_message.axes[1]
            t_yaw = joy_message.axes[2]
            throttle = joy_message.axes[3]
            return t_roll, t_pitch, t_yaw, throttle        

    def loop(self, timestamp, quat):
    

                
    
        return
        

        # Read control from usb joystick
     
    
        # Make ros message and publish it
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes[0] = 0
        
        self.joystick_control_pub.publish(msg)

if __name__ == "__main__":
    joy_controler = JoyRosQuadController()
    joy_controler.loop()
