#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include "/home/macs/catkin_ws/src/youbot_Kinematics/src/youbot_kinematics.h"

using namespace std;

class SubscriberAndPublish
{
public:
	void subscriberCB(const sensor_msgs::Joy::ConstPtr& joy)
	{
		geometry_msgs::Twist base_velocity;    // base velocity
        sensor_msgs::JointState joint_state;   // youbot joint state
        JointPose joint_pose;        // arm joint position
        if(joy->buttons[10] == 1)   step_size = 0.02;   // larger step size
        else    step_size = 0.005;
        if(joy->buttons[9] == 1)    speed_mult = 2;     // higher moving speed
        else    speed_mult = 1;
        if(joy->buttons[4] == 1)   //LB pressed
        {
            theta = 0;
            r = 0.244;
            target_position.z = -0.241;
        }
        if (joy->buttons[5] == 1)   //RB pressed
        {
            theta = 0;
            r = 0.008;
            target_position.z = 0.482;
        }
        if (joy->buttons[7] == 1)   //start pressed
        {
            theta = 2.787;
            r = 0.258556;
            target_position.z = -0.06668;
        }
        base_velocity.linear.x = joy->axes[1]*0.15*speed_mult;
        base_velocity.linear.y = joy->axes[0]*0.15*speed_mult;
        r += joy->axes[4]*step_size;
        theta += joy->axes[3]*step_size*5;
        if(joy->buttons[0] == 1)   // A pressed
        {
            base_velocity.angular.z = -(joy->axes[2]-joy->axes[5])*0.15*speed_mult;
        }
        else
        {
            target_position.z += -(joy->axes[2]-joy->axes[5])*0.5*step_size;
        }
        if(joy->buttons[2] == 1)    // X pressed
        {
            gripper_position = 0.01;   // TODO: determain the max value
        }
        if(joy->buttons[1] == 1)   // B pressed
        {
            gripper_position = 0;
        }
        target_position.x = r*cos(theta);
        target_position.y = r*sin(theta);
        ROS_INFO("target position:[%f,%f,%f]",theta,r,target_position.z);
        joint_pose = youbot.getJointPose(target_position);
        joint_state = writeJointStateMsg(joint_pose,gripper_position);

        twist_pub.publish(base_velocity);
        joint_state_pub.publish(joint_state);
	}

    sensor_msgs::JointState writeJointStateMsg(JointPose joint_pose, double gripper_position)
    {
        sensor_msgs::JointState msg;
        vector<std::string> name_list{"arm_fake_joint_1","arm_fake_joint_2","arm_fake_joint_3","arm_fake_joint_4","arm_fake_joint_5","gripper_fake_finger_joint_l","gripper_fake_finger_joint_r"};
        vector<double> position_list{joint_pose.j1,joint_pose.j2,joint_pose.j3,joint_pose.j4,joint_pose.j5,gripper_position,gripper_position};
        for (auto r : name_list)
        {
            msg.name.push_back(r);
        }
        for (auto r : position_list)
        {
            msg.position.push_back(r);
        }
        msg.header.stamp = ros::Time::now();
        return msg;
    }
private:
	ros::NodeHandle n;
	ros::Publisher twist_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber sub_to_test=n.subscribe<sensor_msgs::Joy>("joy", 1, &SubscriberAndPublish::subscriberCB, this);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states_target",1);
    YoubotKinematics youbot;
    Point3D target_position{0.2,0,0};
    double gripper_position;     // gripper position
    double step_size = 0.005;
    double theta=0;
    double r=0.2;
    double speed_mult = 1;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "youbot_joystick_controller");
	
	SubscriberAndPublish SAPObject;

	ros::spin();
	return 0;
}