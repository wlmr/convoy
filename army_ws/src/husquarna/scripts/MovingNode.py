#!/usr/bin/python

import roslib
import rospy
from collections import namedtuple

"""
linear: 
  x: 0.3
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 1.0
---
linear: 
  x: 0.3
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: -1.0
---
"""

coord_triple = namedtuple('coord_triple', 'x y z')
movement = namedtuple('movement', 'linear angular')
forward_left = movement(coord_triple(0.3, 0, 0), coord_triple(0, 0, 1))
forward_right = movement(coord_triple(0.3, 0, 0), coord_triple(0, 0, -1))



from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from time import sleep


MANUAL_MODE = 0x90
RANDOM_MODE = 0x91
PARK_MODE   = 0x100
START_LOOP  = 0x111
STOP_LOOP   = 0x17


def set_mode(m):
    new_mode = UInt16()
    new_mode.data = m
    mode_pub.publish(new_mode)


def set_movement(m = None):
    t = Twist()
    if m is not None:
        t.angular.x = m.angular.x; t.angular.y = m.angular.y; t.angular.z = m.angular.z
        t.linear.x = m.linear.x; t.linear.y = m.linear.y; t.linear.z = m.linear.z
    vel_pub.publish(t)


def manual_mode():
  from subprocess import Popen, PIPE
  command = "rostopic pub -1 /cmd_mode std_msgs/UInt16 -- '144'".split()
  process = Popen(command, stdout=PIPE, stderr=PIPE)
  stdout, stderr = process.communicate()
  #print(stdout)

def random_mode():
  from subprocess import Popen, PIPE
  command = "rostopic pub -1 /cmd_mode std_msgs/UInt16 -- '145'".split()
  process = Popen(command, stdout=PIPE, stderr=PIPE)
  stdout, stderr = process.communicate()
  #print(stdout)


if __name__=="__main__":
    rospy.init_node('moving_node')
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    mode_pub = rospy.Publisher('cmd_mode', UInt16, queue_size = 10)
    #set_mode(START_LOOP)
    x = 1
    y = 1
    status = 0
    speed = 1
    twist = Twist()
    print("Setting manual mode")
    retries = 0

    while mode_pub.get_num_connections() < 1:
      print("Number of subscribers: %s" % mode_pub.get_num_connections())
      print("Waiting for subscribers")
      sleep(1)
      if retries > 10:
        break
      retries += 1

    set_mode(MANUAL_MODE)

    #manual_mode()
    try:    
        sleep(2)
        print("Moving forward-left")
        set_movement(forward_left)
        sleep(2)
        print("Moving forward-right")
        set_movement(forward_right)
        sleep(2)
    except Exception as e:
        print(e)
    finally:
        print("Stopping...")
        set_movement()
        #random_mode()
        #set_mode(STOP_LOOP)
        set_mode(RANDOM_MODE)


