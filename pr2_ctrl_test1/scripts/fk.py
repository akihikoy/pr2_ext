#!/usr/bin/python
# Test to get end-effector position
# Before running this script, launch:
#   rosrun planning_environment environment_server
#   roslaunch pr2_arm_navigation_kinematics right_arm_collision_free_ik.launch
#   roslaunch pr2_arm_navigation_kinematics left_arm_collision_free_ik.launch
# OR launch:
#   roslaunch pr2_ctrl_test1 pr2_kin.launch

import sys
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
import kinematics_msgs.srv
import pr2_controllers_msgs.msg
import arm_navigation_msgs.srv

ARM_RIGHT = 0
ARM_LEFT = 1

whicharm = ARM_RIGHT
is_mann = False


rospy.init_node('fk_test')


#There must be a planning scene or FK / IK crashes
def setupPlanningScene():
  print 'Waiting for set planning scene service...'
  rospy.wait_for_service('/environment_server/set_planning_scene_diff')
  setPlan = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
  req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
  setPlan(req)
  print 'OK'

setupPlanningScene()

#Set up right/left arm variables
if(whicharm == 0):
    gripper_topic_name = '/r_gripper_controller/state'
    fk_serv_name = '/pr2_right_arm_kinematics/get_fk'
    fk_joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
    link_name = 'r_wrist_roll_link'
    mann_pos_topic_name = '/r_arm_controller_loose/state'
    cont_pos_topic_name = '/r_arm_controller/state'       
else:
    gripper_topic_name = '/l_gripper_controller/state'
    fk_serv_name = '/pr2_left_arm_kinematics/get_fk'
    fk_joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
    link_name = 'l_wrist_roll_link'
    mann_pos_topic_name = '/l_arm_controller_loose/state'
    cont_pos_topic_name = '/l_arm_controller/state' 

if is_mann:
    pos_topic_name = mann_pos_topic_name
else:
    pos_topic_name = cont_pos_topic_name

print 'Waiting for forward kinematics service...'
rospy.wait_for_service(fk_serv_name)
getPosFK = rospy.ServiceProxy(fk_serv_name, kinematics_msgs.srv.GetPositionFK, persistent=False)
print "OK"

FKreq = kinematics_msgs.srv.GetPositionFKRequest()
FKreq.header.frame_id = "torso_lift_link"
FKreq.fk_link_names = [link_name]
FKreq.robot_state.joint_state.name = fk_joints

#time.sleep(1)

def fk_callback(msg):
  #Do forward kinematics on the current joint angles
  global FKreq,getPosFK
  q = msg.actual.positions
  FKreq.robot_state.joint_state.position =  q
  print "Joint angles: "+str(q)
  #print FKreq
  #print getPosFK
  try:
      p = getPosFK(FKreq)
      print p
  except rospy.ServiceException, e:
      print "FK service failure: %s" % str(e) 
      #print FKreq
      sys.exit(0)
  print '------------'
  time.sleep(0.05)



#rospy.init_node('fk_test')


# test fk
q = [0.0]*7
FKreq.robot_state.joint_state.position =  q
print "Joint angles: "+str(q)
#print FKreq
print getPosFK
try:
    p = getPosFK(FKreq)
    print p
except rospy.ServiceException, e:
    print "FK service failure: %s" % str(e) 
    sys.exit(0)
print "-------"


rospy.Subscriber(pos_topic_name, pr2_controllers_msgs.msg.JointTrajectoryControllerState, fk_callback)

rospy.spin()
