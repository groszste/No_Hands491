#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <tf/transform_datatypes.h>

ros::Publisher vel_pub;
ros::Subscriber imu_sub;

geometry_msgs::Twist my_twist;

double roll, pitch, yaw;

void start_from_stop(){
  my_twist.linear.x = 0.5;
  my_twist.angular.z = 1;
  vel_pub.publish(my_twist);
  ros::Duration(1).sleep();
}

void stop(void){
	
	my_twist.linear.x = 0.0;
	my_twist.linear.y = 0.0;
	my_twist.linear.z = 0.0;
	my_twist.angular.x = 0.0;
	my_twist.angular.y = 0.0;
	my_twist.angular.z = 0.0;
	vel_pub.publish(my_twist); 
  ros::Duration(3).sleep();
  start_from_stop();
}

void start(void)
{
  if(yaw > -0.2 and yaw < 0.2){
		ROS_INFO("yaw less than threshold \n");
		stop();
  } else {
	  my_twist.linear.x = 0.5;
	  my_twist.angular.z = 1;
	  vel_pub.publish(my_twist); 
  }
}

void get_yaw_from_imu (const sensor_msgs::ImuConstPtr& msg)
{
	tf::Quaternion q(
	    msg->orientation.x, //orientation_covariance
	    msg->orientation.y,
	    msg->orientation.z,
	    msg->orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
  ROS_INFO("yaw value: %f \n", yaw);
  
  start();
}

int main(int argc, char **argv)
{
	// ROS environment	
	ros::init(argc, argv, "imu_circle");
	ros::NodeHandle n;

  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 2);
  imu_sub = n.subscribe("imu", 2, get_yaw_from_imu);

  //ROS_INFO("yaw: %f \n", yaw);

  //start();

  ros::spin();

  return 0;
}
