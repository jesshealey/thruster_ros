#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from gpiozero import Servo
from time import sleep
import time
from sensor_msgs.msg import Joy
from thruster_ros.msg import MotorCommand

class JoyListener:

    def __init__(self):
        self.vertical = 0
        self.horizontal = 0
        self.thrust_publisher = rospy.Publisher("/motor_cmd", MotorCommand)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)

    #callback function to get joystick values
    def joy_cb(self, msg): 
        self.vertical = msg.axes[1]
        self.horizontal = msg.axes[3]

        axis = self.movement_axis(self.vertical, self.horizontal)
        direction = self.movement_direction(axis)

        self.send_motor_cmd(direction)

    #choosing an axis to move
    def movement_axis(self, vertical, horizontal):
        if vertical < horizontal:
            return "horizontal"
        elif vertical > horizontal:
            return "vertical"
        elif vertical == horizontal:
            return "vertical"

    #chosing direction (fwd/bkwd or left/right)
    def movement_direction(self, axis):
        if axis == "horizontal":
            if self.horizontal > 0:
                return "right"
            elif self.horizontal < 0:
                return "left"
            elif self.horizontal == 0:
                return "stop"
        elif axis == "vertical":
            if self.vertical > 0:
                return "up"
            elif self.vertical < 0:
                return "down"
            elif self.vertical == 0:
                return "stop"

    #send direction to props
    def send_motor_cmd(self, direction):
        msg = MotorCommand()
        
        if direction == "right":
            msg.left = -0.25
            msg.right = self.horizontal
        elif direction == "left":
            msg.right = -0.25
            msg.left = self.horizontal
        elif direction == "stop":
            msg.right = -0.25
            msg.left = -0.25
        elif direction == "up":
            msg.right = self.vertical
            msg.left = self.vertical
        elif direction == "down":
            msg.right = self.vertical
            msg.left = self.vertical

        self.thrust_publisher.publish(msg)


class MotorDriver:

    #function to set up motors and send position to motor controller
    def __init__(self):
        
        self.left_pin = rospy.get_param('left_pin', 12)
        self.right_pin = rospy.get_param('right_pin', 13)
        self.left_servo = Servo(self.left_pin)
        self.right_servo = Servo(self.right_pin)

        self.motor_startup(self.left_servo)
        self.motor_startup(self.right_servo)

        self.motor_cmd_sub = rospy.Subscriber("/motor_cmd", MotorCommand, self.motor_cmd_cb)
        
    def motor_startup(self, motor):
        
        motor.max()
        time.sleep(2)
        motor.min()
        time.sleep(1)
        motor.mid()
        motor.value = -0.25

    def motor_cmd_cb(self, msg):
        self.left_servo.value = msg.left
        self.right_servo.value = msg.right
    
if __name__ == '__main__': 
    try:
        rospy.init_node('TeleopNode')
        joy_listener = JoyListener()        
        motor_driver = MotorDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
