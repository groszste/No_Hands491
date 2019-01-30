#!/usr/bin/env python

import rospy
from math import cos, pi, sin, sqrt
from geometry_msgs.msg import Twist

def fig_8():
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size =10)
    #rate = rospy.Rate(4.)


    traj = Twist()

    flip = True

    while not rospy.is_shutdown():
        
        if flip:
            traj.angular.z = 90*pi/180
        else:
            traj.angular.z = -90*pi/180
        traj.linear.x = 0.5

        pub.publish(traj)

        if flip:
            flip = False
        else:
            flip = True

        rospy.sleep(4.)

if __name__ == '__main__':
    rospy.init_node('figure8_node')
    try:
        fig_8()
    except rospy.ROSInterruptException:
        pass
        


    

