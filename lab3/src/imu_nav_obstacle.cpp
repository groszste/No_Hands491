#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

ros::Publisher vel_pub;
ros::Subscriber odom_sub;
ros::Subscriber imu_sub;

geometry_msgs::Twist my_twist;

double roll, pitch, yaw;

double x = 0.0;
double y = 0.0;
double inc_x, inc_y;
double angle_to_goal;

geometry_msgs::Point final_goal;
geometry_msgs::Point mid_goal;
geometry_msgs::Point current_goal;

//declare functions
void check_goal_reached();
void move_to_current_goal();
void get_yaw_from_imu(const sensor_msgs::ImuConstPtr& msg);
void stop();
void get_position(const nav_msgs::Odometry::ConstPtr& msg);

//function definitions

void get_position(const nav_msgs::Odometry::ConstPtr& msg)
{
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
}

void stop(){
	
	my_twist.linear.x = 0.0;
	my_twist.linear.y = 0.0;
	my_twist.linear.z = 0.0;
	my_twist.angular.x = 0.0;
	my_twist.angular.y = 0.0;
	my_twist.angular.z = 0.0;

	vel_pub.publish(my_twist); 
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
  ROS_INFO("Current Pos x: %f y: %f \n", x, y);
	ROS_INFO("Angle to goal: %f \n", angle_to_goal);
	ROS_INFO("Current goal: x: %f y: %f \n", current_goal.x, current_goal.y);
	ROS_INFO("yaw: %f \n", yaw);
  
  move_to_current_goal();
}

void move_to_current_goal(){
  inc_x = current_goal.x - x;
	inc_y = current_goal.y - y;

	angle_to_goal = atan2(inc_y, inc_x);

	if (angle_to_goal - yaw > 0.02){ // || angle_to_goal - yaw < -0.03
		my_twist.linear.x = 0.0;
		my_twist.linear.y = 0.0;
		my_twist.linear.z = 0.0;
		my_twist.angular.x = 0.0;
		my_twist.angular.y = 0.0;
		my_twist.angular.z = 0.1;
		vel_pub.publish(my_twist);
	} else if (angle_to_goal - yaw < -0.02) {
		my_twist.linear.x = 0.0;
		my_twist.linear.y = 0.0;
		my_twist.linear.z = 0.0;
		my_twist.angular.x = 0.0;
		my_twist.angular.y = 0.0;
		my_twist.angular.z = -0.1;
		vel_pub.publish(my_twist);
	} else {
		my_twist.linear.x = 0.2;
		my_twist.linear.y = 0.0;
		my_twist.linear.z = 0.0;
		my_twist.angular.x = 0.0;
		my_twist.angular.y = 0.0;
		my_twist.angular.z = 0.0;
		vel_pub.publish(my_twist);
	}
  check_goal_reached();
}

void check_goal_reached(){
  if(x > current_goal.x - 0.2 && x < current_goal.x + 0.2 && y > current_goal.y - 0.2 && y < current_goal.y + 0.2){
			
		stop();
			
		current_goal.x = final_goal.x;
		current_goal.y = final_goal.y;

		//Duration(1.0).sleep();
	}
}

int main(int argc, char **argv)
{
	// ROS environment	
	ros::init(argc, argv, "imu_nav");
	ros::NodeHandle n;
	
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 2);
	odom_sub = n.subscribe("odom", 2, get_position);
	imu_sub = n.subscribe("imu", 2, get_yaw_from_imu);

	final_goal.x = 2;
	final_goal.y =2;

	mid_goal.x = 0;
	mid_goal.y = 2;

	current_goal.x = mid_goal.x;
	current_goal.y = mid_goal.y;

	ros::spin();

	//Duration(0.5).sleep();

	return 0;

}

