from jetbot import Robot
import time

#pip install traitlets
#pip install Adafruit-MotorHAT
#pip install sparkfun-qwiic

robot = Robot()

robot.left(0.3)
time.sleep(0.5)
robot.stop()

