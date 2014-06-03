#!/usr/bin/python
import sys
import time
import math
import numpy as np
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
import trajectory_msgs.msg
import pr2_controllers_msgs.msg
import actionlib
#from geometry_msgs.msg import *
#from pr2_mechanism_controllers.msg import *


class Test:
  def __init__(self):

    self.max_pan= 2.7
    self.max_tilt= 1.4
    self.min_tilt= -0.4
    self.tilt_step= 0.3
    self.pan_step= 0.3
    self.head_pub = rospy.Publisher("/head_traj_controller/command", trajectory_msgs.msg.JointTrajectory)

    self.head_joint_names= ['head_pan_joint', 'head_tilt_joint']

    self.time_step= 0.1

  def Curve(self,t):
    #w= 4.0
    w= 0.5
    return 0.1*(1.0-math.cos(w*t))

  def CurveD(self,t):
    dt= 0.0001
    return (self.Curve(t+dt)-self.Curve(t))/dt


  #Move to original pose
  def MoveToBase(self):
    traj= trajectory_msgs.msg.JointTrajectory()
    traj.joint_names= self.head_joint_names
    traj.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

    traj.points[0].time_from_start= rospy.Duration(2.0)
    traj.points[0].positions= [0.0]*2
    traj.points[0].velocities= [0.0]*2
    traj.header.stamp = rospy.Time.now()
    print traj
    self.head_pub.publish(traj)

  def Control11(self):
    t= 0.0
    dt= self.time_step
    base= [0.0]*2
    traj= trajectory_msgs.msg.JointTrajectory()
    traj.joint_names= self.head_joint_names
    traj.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    while t<3.14159*4.0:
      t= t+dt
      traj.points[0].time_from_start= rospy.Duration(dt)
      traj.points[0].positions= base + np.array([self.Curve(t)]*2)
      traj.points[0].velocities= [self.CurveD(t)]*2

      traj.header.stamp = rospy.Time.now()
      self.head_pub.publish(traj)

      #start_time= rospy.Time.now()
      while rospy.Time.now() < traj.header.stamp + rospy.Duration(dt):
        time.sleep(dt*0.02)


rospy.init_node('head_ctrl')

#Before using rospy.Time.now(), this is necessary!!
time.sleep(0.1)


t= Test()
time.sleep(0.2)

#t.MoveToBase()

#print "Control11..."
t.Control11()
#time.sleep(0.8)

#print "Control12..."
#t.Control12()
#time.sleep(0.8)

#print "Control21..."
#t.Control21()
#time.sleep(0.5)

#print "Control22..."
#t.Control22()
#time.sleep(0.5)

#rospy.spin()

#print "Done"
