#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>

#define PI 3.141592

bool ctrl_flag=true;						// control flag

float rad;							// radius
float th;							// theta

float vel=6000.0;						// motor velocity

float left_v;
float right_v;

int mot1_v;
int mot2_v;

class publisher_subscriber					// class for pub, sub
{
	public:						// public member declaration
		publisher_subscriber()		// substitute topics to publisher & subscriber
		{
			vel_l_pub=nh.advertise<std_msgs::Float32>("vel_l", 1);
			vel_r_pub=nh.advertise<std_msgs::Float32>("vel_r", 1);
			flag_sub=nh.subscribe("ctrl_flag", 1000, &publisher_subscriber::ctrl_flag_callback, this);
			joy_sub=nh.subscribe("cmd_joy", 1000, &publisher_subscriber::joy_callback, this);
		}
		
		void ctrl_flag_callback(const std_msgs::Bool::ConstPtr& flag) // ctrl flag call back func.
		{
			if(flag->data==true)
    			{
    				ctrl_flag=!ctrl_flag;
    				ROS_INFO("%s", ctrl_flag ? "true" : "false");
    			}
		}
		
		void joy_callback(const geometry_msgs::Twist::ConstPtr& msg)
		{	
			std_msgs::Float32 vel_l;			// Float32 data type var.
			std_msgs::Float32 vel_r;			// Float32 data type var.
									
			rad=sqrt((msg->linear.x)*(msg->linear.x)+(msg->angular.z)*(msg->angular.z));	// radius
			th=atan2(msg->linear.x, msg->angular.z);	// theta
	
			if((th>=0.0) && (th<(PI/2)))			// 1st quadrant(up, left)
			{
				left_v=th*2*vel/(PI/2)-vel;
				right_v=vel;
			}
			else if((th>=(PI/2)) && (th<PI))		// 2nd quadrant(up, right)
			{
				left_v=vel;
				right_v=-th*2*vel/(PI/2)+vel*3;
			}
			else if((th>=(-PI/2)) && (th<0))		// 3rd quadrant(down, left)
			{
				left_v=th*2*vel/(PI/2)+vel;
				right_v=-vel;
			}
			else if((th>=(-PI)) && (th<(-PI/2)))		// 4th quadrant(down, right)
			{
				left_v=-vel;
				right_v=-th*2*vel/(PI/2)-vel*3;
			}
	
			left_v*=rad;
			right_v*=rad;
			
			vel_l.data=left_v;				// left motor velocity
    			vel_r.data=right_v;				// right motor velocity
    			
    			ROS_INFO("left: %f", vel_l.data);
    			ROS_INFO("right: %f", vel_r.data);
			
			if(ctrl_flag==true)
			{
				vel_l_pub.publish(vel_l);		// publish the pos topic
				vel_r_pub.publish(vel_r);		// publish the pos topic
				ROS_INFO("sent command");
			}
		}
		
	private:					// private member declaration
	  	ros::NodeHandle nh; 			// declare node handle
		ros::Publisher vel_l_pub;		// declare publisher
		ros::Publisher vel_r_pub;
  		ros::Subscriber joy_sub;		// declare subscriber
  		ros::Subscriber flag_sub;
};

int main(int argc, char **argv)			// main function
{
	ros::init(argc, argv, "mobile_joy_ctrl");	// ros initialization
	publisher_subscriber pub_sub;			// class object delaration
	
	ros::Rate loop_rate(10);			// set 10ms loop rate
	
	while(ros::ok())				// while loop
	{
		ros::spinOnce();			// run ros once 
		
		loop_rate.sleep();			// sleep to keep the loop rate
	}
	
	return 0;					
}
