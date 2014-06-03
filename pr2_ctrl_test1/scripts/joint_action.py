#!/usr/bin/python
import sys
import time
import math
import numpy as np
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pr2_mechanism_controllers')
import rospy
import trajectory_msgs.msg
import pr2_controllers_msgs.msg
import actionlib
#from geometry_msgs.msg import *
#from pr2_mechanism_controllers.msg import *


class Test:
  def __init__(self):
    #self.l_pub = rospy.Publisher("/l_arm_controller/command", trajectory_msgs.msg.JointTrajectory)
    self.traj_client = actionlib.SimpleActionClient('/l_arm_controller/joint_trajectory_action', pr2_controllers_msgs.msg.JointTrajectoryAction)
    while not self.traj_client.wait_for_server(rospy.Duration(5.0)):
        print "Waiting for the joint_trajectory_action server..."
    print "Connected to joint_trajectory_action server"


    rospy.Subscriber("/l_arm_controller/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.LeftJointStateCallback)

    self.joint_positions= [[0.0]*7]*2
    self.joint_names= [[], ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']]

    self.time_step= 0.2
    self.over_target= False

    self.ljscallback_init= True
    self.fp_state= file('log.dat','w')
    self.fp_goal= file('log_goal.dat','w')

  def Curve(self,t):
    #w= 4.0
    w= 0.5
    return 0.1*(math.cos(w*t)-1.0)

  def CurveD(self,t):
    dt= 0.0001
    return (self.Curve(t+dt)-self.Curve(t))/dt

  def LeftJointStateCallback(self,msg):
    if self.ljscallback_init:
      self.ljscallback_start_time= rospy.Time.now()
      self.ljscallback_init= False

    self.joint_positions[1]= np.array(msg.actual.positions)
    self.last_l_arm_controller_state= msg
    self.fp_state.write('%f %s %s\n' % ( (rospy.Time.now()-self.ljscallback_start_time).to_sec(), ' '.join(map(str,msg.actual.positions)), ' '.join(map(str,msg.actual.velocities)) ))

  #Move to original pose
  def MoveToBase(self):
    goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names= self.joint_names[1];
    goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

    goal.trajectory.points[0].time_from_start= rospy.Duration(2.0)
    goal.trajectory.points[0].positions= [0.0]*7
    goal.trajectory.points[0].velocities= [0.0]*7
    self.traj_client.send_goal(goal)
    self.traj_client.wait_for_result()

  def Control11(self):
    t= 0.0
    dt= self.time_step
    #base= self.joint_positions[1]
    base= [0.0]*7
    goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names= self.joint_names[1];
    goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    if self.over_target:
      goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    while t<3.14159*4.0:
      t= t+dt
      goal.trajectory.points[0].time_from_start= rospy.Duration(dt)
      goal.trajectory.points[0].positions= base + np.array([self.Curve(t)]*7)
      if self.over_target:
        goal.trajectory.points[1].time_from_start= rospy.Duration(dt*2)
        goal.trajectory.points[1].positions= base + np.array([self.Curve(t+dt)]*7)
      goal.trajectory.header.stamp= rospy.Time.now()
      self.traj_client.send_goal(goal)
      #self.traj_client.wait_for_result()
      start_time= rospy.get_rostime()
      while rospy.get_rostime() < start_time + rospy.Duration(dt):
        time.sleep(dt*0.1)

  def Control12(self):
    t= 0.0
    dt= self.time_step
    #base= self.joint_positions[1]
    base= [0.0]*7
    goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names= self.joint_names[1];
    goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    if self.over_target:
      goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    while t<3.14159*4.0:
      t= t+dt
      goal.trajectory.points[0].time_from_start= rospy.Duration(dt)
      goal.trajectory.points[0].positions= base + np.array([self.Curve(t)]*7)
      #goal.trajectory.points[0].velocities= [0.0]*7
      goal.trajectory.points[0].velocities= [self.CurveD(t)]*7
      if self.over_target:
        goal.trajectory.points[1].time_from_start= rospy.Duration(dt*2)
        goal.trajectory.points[1].positions= base + np.array([self.Curve(t+dt)]*7)
        #goal.trajectory.points[1].velocities= [0.0]*7
        goal.trajectory.points[1].velocities= [self.CurveD(t+dt)]*7
      print goal.trajectory.points
      goal.trajectory.header.stamp= rospy.Time.now()
      self.traj_client.send_goal(goal)
      #self.traj_client.wait_for_result()
      start_time= rospy.get_rostime()
      while rospy.get_rostime() < start_time + rospy.Duration(dt):
        time.sleep(dt*0.1)

  #def Control21(self):
    #t= 0.0
    #dt= self.time_step
    ##base= self.joint_positions[1]
    #base= [0.0]*7
    #target= base
    #vel= np.array([0.05]*7)
    #cmd= trajectory_msgs.msg.JointTrajectory()
    #cmd.header.stamp= self.last_l_arm_controller_state.header.stamp
    #cmd.joint_names= self.joint_names[1]
    #cmd.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    #while t<3.14159:
      #t= t+dt
      #target= target+dt*vel
      #cmd.points[0].time_from_start= rospy.Duration(dt)
      #cmd.points[0].positions= target
      #self.l_pub.publish(cmd)
      #start_time= rospy.get_rostime()
      #while rospy.get_rostime() < start_time + rospy.Duration(dt):
        #time.sleep(dt*0.1)

    #time.sleep(0.3)
    #self.MoveToBase()

  #def Control22(self):
    #t= 0.0
    #dt= self.time_step
    ##base= self.joint_positions[1]
    #base= [0.0]*7
    #target= base
    #vel= np.array([0.05]*7)
    #cmd= trajectory_msgs.msg.JointTrajectory()
    #cmd.header.stamp= self.last_l_arm_controller_state.header.stamp
    #cmd.joint_names= self.joint_names[1]
    #cmd.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    #while t<3.14159:
      #t= t+dt
      #target= target+dt*vel
      #cmd.points[0].time_from_start= rospy.Duration(dt)
      #cmd.points[0].velocities= vel
      #cmd.points[0].positions= target
      #self.l_pub.publish(cmd)
      #start_time= rospy.get_rostime()
      #while rospy.get_rostime() < start_time + rospy.Duration(dt):
        #time.sleep(dt*0.1)

    #time.sleep(0.3)
    #self.MoveToBase()


rospy.init_node('joint_ctrl')

#Before using rospy.Time.now(), this is necessary!!
time.sleep(0.1)


t= Test()
time.sleep(0.2)

#t.MoveToBase()

#print "Control11..."
t.Control11()
time.sleep(0.8)

#print "Control12..."
t.Control12()
time.sleep(0.8)

#print "Control21..."
#t.Control21()
#time.sleep(0.5)

#print "Control22..."
#t.Control22()
#time.sleep(0.5)

#rospy.spin()

#print "Done"
