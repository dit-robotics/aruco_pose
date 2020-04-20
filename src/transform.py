#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from aruco_pose.msg import MarkerArray
import numpy as np
import rigid_transform_3D
from numpy import *
from math import sqrt


def callback(data):

  if data.markers:
        rospy.loginfo(data.markers[0].id)
	rospy.loginfo(data.markers[1].id)
	rospy.loginfo(data.markers[2].id)
	rospy.loginfo(data.markers[3].id)
	rospy.loginfo(data.markers[4].id)
	rospy.loginfo(data.markers[5].id)
	rospy.loginfo(data.markers[6].id)
      
	for i in range(7):
	  k =  data.markers[i].id
	  if k == 50:
  	    a = i
	  elif k == 54:
	    b = i
          elif k == 58:
	    c = i
	  elif k == 60:
	    d = i
          elif k == 64:
	    e = i
	  elif k == 68:
	    f = i
          elif k == 70:
	    g = i

	#print(a,b,c,d,e,f)
	
	A = np.mat([[data.markers[a].pose.position.x,data.markers[b].pose.position.x,data.markers[c].pose.position.x,data.markers[d].pose.position.x,data.markers[e].pose.position.x,data.markers[f].pose.position.x,data.markers[g].pose.position.x],[data.markers[a].pose.position.y,data.markers[b].pose.position.y,data.markers[c].pose.position.y,data.markers[d].pose.position.y,data.markers[e].pose.position.y,data.markers[f].pose.position.y,data.markers[g].pose.position.y],[data.markers[a].pose.position.z,data.markers[b].pose.position.z,data.markers[c].pose.position.z,data.markers[d].pose.position.z,data.markers[e].pose.position.z,data.markers[f].pose.position.z,data.markers[g].pose.position.z]])
	
	B = np.mat([[300,400,400,500,600,600,900],[1500,1010,1990,1500,1010,1990,1500],[0,0,0,0,0,0,0]])	
	print(A)
	print(B)

        R, t = rigid_transform_3D.rigid_transform_3D(A, B)
        print(R)
        print(t)

	with open('/home/ubuntu/catkin_ws/src/clever/aruco_pose/cfg/R_matrix','wb') as f:
            for line in R:
        	np.savetxt(f, line, fmt='%.8f')
	
	#rospy.loginfo(data.markers[0].pose.position.x)
	#rospy.loginfo(data.markers[0].pose.position.y)
	#rospy.loginfo(data.markers[0].pose.position.z)

  else:
	rospy.loginfo("none")


    
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("aruco_detect/markers",MarkerArray , callback)

    rospy.spin()

if __name__ == '__main__':
    listener()


