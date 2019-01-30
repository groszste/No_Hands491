#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AvoidObj():
    def __init__(self):
        self.LIDAR_ERR = 0.05
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
        self.run()

    def read_scan(self):
        msg = rospy.wait_for_message("scan",LaserScan)
        self.scan_filter = []
        for i in range(0,360):
            if msg.ranges[i] >= self.LIDAR_ERR:
                self.scan_filter.append(msg.ranges[i])

    def run(self):
        self.traj = Twist()
        while not rospy.is_shutdown():
            self.read_scan()

            if min(self.scan_filter) < 0.2:
                self.traj.linear.x = 0.0
                self.traj.angular.z = 0.0
                rospy.loginfo("Object Detected - Stopping")

            else:
                self.traj.linear.x = 0.05
                self.traj.angular.z = 0.0
                rospy.loginfo('Closest object detected at {}'.format(min(self.scan_filter))) 
            self.cmd_pub.publish(self.traj)      

def main():
    rospy.init_node('ObjAvoidence')
    try:
        AvoidObj()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

