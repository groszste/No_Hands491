#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion

class report():
    def __init__(self):
        self.read_odm()
    
    def read_odm(self):

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("odom",Odometry)

            locx = msg.pose.pose.position.x
            locy = msg.pose.pose.position.y
            locz = msg.pose.pose.position.z

            if locx > 1.5 or locy > 1.5 or locz > 1.5 or locx < -1.5 or locy < -1.5 or locz < -1.5 :
                rospy.loginfo("Turtle has left the 3 by 3 square around origin")
            else:
                rospy.loginfo("Turtle is within a 3 by 3 from the origin")
            
            rospy.sleep(1.)

def main():
    rospy.init_node('ReportFence')
    try:
        report()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
        