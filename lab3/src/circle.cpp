#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

using namespace ros;

Publisher cir_pub;


void move_in_circle(void)
{
	geometry_msgs::Twist my_twist;
	my_twist.linear.x = 0.5;
	my_twist.angular.z = 2;
	cir_pub.publish(my_twist); 
}


void stop(void){
	geometry_msgs::Twist my_twist;
	
	my_twist.linear.x = 0.0;
	my_twist.linear.y = 0.0;
	my_twist.linear.z = 0.0;
	my_twist.angular.x = 0.0;
	my_twist.angular.y = 0.0;
	my_twist.angular.z = 0.0;

	cir_pub.publish(my_twist); 
}

int main(int argc, char **argv)
{
	// ROS environment	
	init(argc, argv, "circle");
	NodeHandle n;
	
	cir_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	while(ok())
	{

		move_in_circle();
		
		ROS_INFO("After circle call \n");
		

		Duration(3.3).sleep();

		ROS_INFO("After 1st sleep call \n");		

		stop();

		ROS_INFO("After stop call \n");

		Duration(3).sleep();
		
		ROS_INFO("After second sleep call \n");
			
		spinOnce();
	}

	return 0;

}

