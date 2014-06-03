#!/usr/bin/python
import sys
import copy
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

# Matlab-like mod function that returns always positive
def Mod(x, y):
  if y==0:  return x
  return x-y*math.floor(x/y)

#Convert radian to [-pi,pi)
def AngleMod1(q):
  return Mod(q+math.pi,math.pi*2.0)-math.pi

#Convert radian to [0,2*pi)
def AngleMod2(q):
  return Mod(q,math.pi*2.0)


#Interpolate (p1,v1)-(p2,v2) of duration with num_p points and store into traj_points
#(p1,v1) is not contained.  traj_points should have the size num_p
def InterpolateLinearly1(traj_points,p1,v1,p2,v2,duration,num_p):
  if len(traj_points)!=num_p:
    rospy.logerr('Error in InterpolateLinearly1: len(traj_points)!=num_p')
    sys.exit(1)
  p= np.array(p1).copy()
  v= np.array(v1).copy()
  dt= duration/float(num_p)
  dp= (p2-p1)/float(num_p)
  dv= (v2-v1)/float(num_p)
  for i in range(num_p):
    p+= dp
    v+= dv
    traj_points[i].time_from_start= rospy.Duration(dt*(i+1.0))
    traj_points[i].positions= p.copy()
    traj_points[i].velocities= v.copy()

#Interpolate (p1)-(p2) of duration with time_step and store into traj_points
#(p1) is not contained.  Velocities are automatically computed
#If rot_adjust is true, each dimension is considered as a revolute joint, and a shorter direction is automatically chosen
#If vel_limits is specified, the velocity is adjusted so that it does not exceed vel_limits.
#When the velocity is modified, the duration is also modified, which is returned
def InterpolateLinearly2(traj_points,p1,p2,duration,time_step,rot_adjust=False, vel_limits=[]):
  traj_points[:]= []
  v= (p2-p1)/duration
  if rot_adjust:
    for d in range(len(p1)):
      delta= p2[d]-p1[d]
      if math.fabs(delta)>math.pi:
        v[d]= AngleMod1(delta)/duration
  if len(vel_limits)>0:
    #Borrowed Scott's velocity modification
    ratios= [math.fabs(v[d]) / vel_limits[d] for d in range(len(v))]
    max_r= max(ratios)
    if max_r>1.0:
      v= v/max_r
      duration= duration*max_r
  t= 0.0
  jp= trajectory_msgs.msg.JointTrajectoryPoint()
  t+= time_step
  while t<duration:
    p= p1+v*t
    jp.time_from_start= rospy.Duration(t)
    jp.positions= p.copy()
    jp.velocities= v.copy()
    traj_points.append(copy.deepcopy(jp))
    t+= time_step
  if t-time_step<=duration:
    p= p1+v*duration
    jp.time_from_start= rospy.Duration(duration)
    jp.positions= p.copy()
    jp.velocities= v.copy()
    traj_points.append(copy.deepcopy(jp))
  return duration


class Test:
  def __init__(self):
    #self.l_pub = rospy.Publisher("/l_arm_controller/command", trajectory_msgs.msg.JointTrajectory)
    self.traj_client = actionlib.SimpleActionClient('/l_arm_controller/joint_trajectory_action', pr2_controllers_msgs.msg.JointTrajectoryAction)
    while not self.traj_client.wait_for_server(rospy.Duration(5.0)):
        print "Waiting for the joint_trajectory_action server..."
    print "Connected to joint_trajectory_action server"

    #self.vel_limits = [0.8, 0.8, 2.0, 2.0, 3.0, 3.0, 10.0]
    #self.vel_limits = [0.2, 0.2, 0.5, 0.5, 1.0, 1.0, 2.0]
    self.vel_limits = []

    rospy.Subscriber("/l_arm_controller/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.LeftJointStateCallback)

    self.joint_positions= [[0.0]*7]*2
    self.joint_velocities= [[0.0]*7]*2
    self.joint_names= [[], ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']]

    self.time_step= 0.1
    #self.time_step= 0.05
    self.num_points= 10

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
    self.joint_velocities[1]= np.array(msg.actual.velocities)
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
    base= np.array([0.0]*7)
    goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    goal.trajectory.header.stamp= rospy.Time.now()
    goal.trajectory.joint_names= self.joint_names[1];
    for i in range(self.num_points):
      goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    g_prev= base.copy()
    while t<3.14159*4.0:
      traj_duration= dt
      t= t+dt
      g= np.array([self.Curve(t)]*7)
      g[4]= AngleMod1(40.0*(g[4]))
      dg= np.array([self.CurveD(t)]*7)
      #InterpolateLinearly1(goal.trajectory.points, self.joint_positions[1],self.joint_velocities[1], g,dg, dt, self.num_points)
      if False:
        InterpolateLinearly1(goal.trajectory.points, g_prev, np.array([self.CurveD(t-dt)]*7), g, np.array([self.CurveD(t)]*7), dt, self.num_points)
      else:
        traj_duration= InterpolateLinearly2(goal.trajectory.points, g_prev, g, dt, dt/float(self.num_points), True, self.vel_limits)

      #print goal.trajectory.points
      for p in goal.trajectory.points:
        self.fp_goal.write('%f %s %s\n' % ( (rospy.get_rostime()-self.ljscallback_start_time+p.time_from_start).to_sec(), ' '.join(map(str,p.positions)), ' '.join(map(str,p.velocities)) ))
      #self.fp_goal.write('%f %s %s\n' % ( (rospy.get_rostime()-self.ljscallback_start_time).to_sec(), ' '.join(map(str,g)), ' '.join(map(str,dg)) ))
      goal.trajectory.header.stamp= rospy.Time.now()
      self.traj_client.send_goal(goal)
      #self.traj_client.wait_for_result()
      start_time= rospy.get_rostime()
      while rospy.get_rostime() < start_time + rospy.Duration(traj_duration):
        time.sleep(traj_duration*0.01)
      g_prev= g

  #def Control12(self):
    #t= 0.0
    #dt= self.time_step
    ##base= self.joint_positions[1]
    #base= [0.0]*7
    #goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    #goal.trajectory.header.stamp = rospy.Time.now()
    #goal.trajectory.joint_names= self.joint_names[1];
    #goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    #if self.over_target:
      #goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    #while t<3.14159*4.0:
      #t= t+dt
      #goal.trajectory.points[0].time_from_start= rospy.Duration(dt)
      #goal.trajectory.points[0].positions= base + np.array([self.Curve(t)]*7)
      ##goal.trajectory.points[0].velocities= [0.0]*7
      #goal.trajectory.points[0].velocities= [self.CurveD(t)]*7
      #if self.over_target:
        #goal.trajectory.points[1].time_from_start= rospy.Duration(dt*2)
        #goal.trajectory.points[1].positions= base + np.array([self.Curve(t+dt)]*7)
        ##goal.trajectory.points[1].velocities= [0.0]*7
        #goal.trajectory.points[1].velocities= [self.CurveD(t+dt)]*7
      #self.traj_client.send_goal(goal)
      #self.traj_client.wait_for_result()
      #start_time= rospy.get_rostime()
      #while rospy.get_rostime() < start_time + rospy.Duration(dt):
        #time.sleep(dt*0.1)

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

t.MoveToBase()
time.sleep(0.2)

#print "Control11..."
t.Control11()
time.sleep(0.8)

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
