#!/usr/bin/python
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pr2_mechanism_controllers')
import rospy
from geometry_msgs.msg import *
from pr2_mechanism_controllers.msg import *

def base_callback(state_msg):
  print rospy.get_rostime()
  #print state_msg

rospy.init_node('mov_base')

base_pub= rospy.Publisher('base_controller/command', Twist)
#base_sub= rospy.Subscriber('base_controller/state', BaseControllerState, base_callback)
time.sleep(1)  # What for?

movement= Twist()
movement.linear.x= 0.3
#movement.angular.z= 2.0
start_time= rospy.get_rostime()
while rospy.get_rostime() < start_time + rospy.Duration(3.0):
  base_pub.publish(movement)
  time.sleep(0.1)
  #rospy.spin()

base_pub.publish(Twist())  # Stop

rospy.spin()
