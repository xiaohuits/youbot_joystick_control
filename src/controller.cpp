#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <stdio.h>

using namespace std;

geometry_msgs::Twist twistSet(const double& lx, const double& ly, const double& az)
{
	geometry_msgs::Twist msg;
	msg.linear.x = lx;
	msg.linear.y = ly;
	msg.angular.z = az;
	return msg;
}

geometry_msgs::Twist movement(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist msg;
	double lstickh = joy->axes[0];
	double lstickv = joy->axes[1];
	double rstickh = joy->axes[3];
//	ROS_INFO(joy->axes);
	msg = twistSet(0, lstickv, lstickh);	
//	arm_position[0] = arm_step_size*joy->axes[4];
	cout << msg << endl;
//	twist_pub.publish(msg);
	return msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber sub_to_test = n.subscribe<sensor_msgs::Joy>("joy", 1, movement);
	ros::spin();
	return 0;
}

	


