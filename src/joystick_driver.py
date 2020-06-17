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

# On xbox gamepad tits are -1 [left, up] and +1 [right, down], 0 in neutral. long buttons are default -1 and +1 fully pressed

class JoyRosQuadController():
    def __init__(self, update_rate=30):
        print("Initializing quadcopter ros joystick controller")
        
        self.joystick_control_pub = rospy.Publisher('quad_teleop', Joy, queue_size=10)
        rospy.init_node('ros_quad_joystick_controller_node')
        self.ros_rate = rospy.Rate(update_rate)

        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print("Initialized gamepad: {}".format(self.joystick.get_name()))
        print("Finished initializing the quadcopter ros joystick controller.")


    def loop(self):
        print("Starting joystick loop.")
        while True:
            pygame.event.pump()
            data = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]

            # Make ros message and publish it
            msg = Joy()
            msg.header.stamp = rospy.Time.now()
            msg.axes = data

            self.joystick_control_pub.publish(msg)

            pygame.event.clear()
            self.ros_rate.sleep()


if __name__ == "__main__":
    joy_controler = JoyRosQuadController()
    joy_controler.loop()
