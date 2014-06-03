#!/usr/bin/python
# Test to use tf for quaternion calculations

import sys
import time
import roslib
roslib.load_manifest('tf')
import rospy
import tf
import numpy as np
import numpy.linalg as la

roll = 0.5
pitch = 0.0
yaw = 0.0
quaternion1 = [0.,0.,0.,1.]
quaternion2 = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
print "quaternion1= ",quaternion1
print "quaternion2= ",quaternion2
print "quaternion2*quaternion1= ",quaternion2*quaternion1, '  # THIS DOES NOT WORK AS EXPECTED'
print "tf.transformations.quaternion_multiply(quaternion2,quaternion1)= ",tf.transformations.quaternion_multiply(quaternion2,quaternion1)

quaternion3 = tf.transformations.quaternion_about_axis(0.5, (1,0.,0.))
print "quaternion3= ",quaternion3
print "tf.transformations.quaternion_multiply(quaternion3,quaternion2)= ",tf.transformations.quaternion_multiply(quaternion3,quaternion2)

q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
print 'q= ',q
p = np.array([1.0,0.5,0.5])
print 'p= ',p
R = tf.transformations.quaternion_matrix(q)[:3,:3]
print 'R= ',R
print 'R*p= ',np.dot(R,p)
print 'R\'= ',R.T
print 'R\'*R= ',np.dot(R.T,R)

print 'list(p)+[0,0,0]= ',list(p)+[0,0,0]

M = tf.transformations.identity_matrix()
M[:3,:3] = R
print 'M= ',M
print 'tf.transformations.quaternion_from_matrix(M)= ',tf.transformations.quaternion_from_matrix(M)



