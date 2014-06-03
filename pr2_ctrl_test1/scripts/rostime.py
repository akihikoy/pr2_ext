#!/usr/bin/python
import sys
import time
import roslib
roslib.load_manifest('rospy')
import rospy

rospy.init_node('joint_ctrl')

t= 0.0
dt= 0.1

#Before using rospy.Time.now(), this is necessary!!
time.sleep(0.1)

print '----------'
d=rospy.Duration(1.0)
print d.to_sec()
print d.to_nsec()

print '----------'
#start_time= rospy.get_rostime()
start_time= rospy.Time.now()
print 'start_time=',start_time.to_sec()
print 'rospy.Time.now()=',rospy.Time.now().to_sec()
print (rospy.Time.now()-start_time).to_sec()
time.sleep(dt)
print (rospy.Time.now()-start_time).to_sec()
time.sleep(dt)
print (rospy.Time.now()-start_time).to_sec()

print '----------'
start_time= rospy.Time.now()
while t<5.0:
  time.sleep(dt)
  t= t+dt
  #print t,' ',(rospy.get_rostime()-start_time).to_sec()
  print t,' ',(rospy.Time.now()-start_time).to_sec()

print '----------'
start_time= rospy.get_rostime()
while rospy.get_rostime() < start_time + rospy.Duration(dt):
  time.sleep(dt*0.1)
print 'duration=',(rospy.Time.now()-start_time).to_sec()

