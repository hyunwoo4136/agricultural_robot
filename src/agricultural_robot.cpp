#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>


///////////////////////////////////////////////////////////////////////////	var. declaration
#define PI 3.141592

std_msgs::String mod_log;							// string for driving mod log publish	
geometry_msgs::Twist vel;							// velocity for command publish
geometry_msgs::Twist sel_vel;						// subscribed velocity
geometry_msgs::Twist obj_vel;
geometry_msgs::Twist con_vel;

bool mod_flag=false;
bool sel_flag=false;
bool obj_flag=false;
bool con_flag=true;






bool ctrl_flag=true;								// control flag

float rad;											// radius
float th;											// theta

//float vel=6000.0;									// motor velocity

float left_v;
float right_v;

int mot1_v;
int mot2_v;


///////////////////////////////////////////////////////////////////////////	sub, pub class
class sub_pub
{
private:
	ros::NodeHandle nh;
	ros::Publisher mod_pub;							// driving mod log publisher
	ros::Publisher vel_pub;							// command velocity publisher
	ros::Subscriber mod_sub;						// module operation command subscriber
	ros::Subscriber sel_sub;						// self driving command subscriber
	ros::Subscriber obj_sub;						// object following command subscriber
	ros::Subscriber con_sub;						// controller command subscriber
	ros::Subscriber sel_cmd_sub;					// self driving velocity subscriber
	ros::Subscriber obj_cmd_sub;					// object following velocity subscruber
	ros::Subscriber con_cmd_sub;					// controller joystick velocity subscriber
	
public:
	sub_pub()										// class constructor
	{
		mod_pub=nh.advertise<std_msgs::String>("/log", 1);
		vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		mod_sub=nh.subscribe("cmd_mod", 10, &sub_pub::mod_callback, this);
		sel_sub=nh.subscribe("cmd_sel", 10, &sub_pub::sel_callback, this);
		obj_sub=nh.subscribe("cmd_obj", 10, &sub_pub::obj_callback, this);
		con_sub=nh.subscribe("cmd_con", 10, &sub_pub::con_callback, this);
		sel_cmd_sub=nh.subscribe("vel_sel", 100, &sub_pub::sel_vel_callback, this);
		obj_cmd_sub=nh.subscribe("vel_obj", 100, &sub_pub::obj_vel_callback, this);
		con_cmd_sub=nh.subscribe("vel_con", 100, &sub_pub::con_vel_callback, this);
	}
	
	void mod_publish()								// driving mod publish func.
	{
		if(sel_flag==true)
			mod_log.data="self driving";
		else if(obj_flag==true)
			mod_log.data="following object";
		else if(con_flag==true)
			mod_log.data="driving with joystick";
		
		mod_pub.publish(mod_log);
	}
	
	void vel_publish()								// velocity publish func.
	{
		vel_pub.publish(vel);
	}
	
	void mod_callback(const std_msgs::Bool::ConstPtr& msg)	// module command call back func.
	{
		if(msg->data==true)
			mod_flag=!mod_flag;
	}
	
	void sel_callback(const std_msgs::Bool::ConstPtr& msg)	// self driving cmd call back func.
	{
		if(msg->data==true)
			sel_flag=!sel_flag;
		
		if(sel_flag==true)
		{
			obj_flag=false;
			con_flag=false;
		}
	}
	
	void obj_callback(const std_msgs::Bool::ConstPtr& msg)	// obj. following cmd call back func.
	{
		if(msg->data==true)
			obj_flag=!obj_flag;
		
		if(obj_flag==true)
		{
			sel_flag=false;
			con_flag=false;
		}
	}
	
	void con_callback(const std_msgs::Bool::ConstPtr& msg)	// controller cmd call back func.
	{
		if(msg->data==true)
			con_flag=!con_flag;
		
		if(con_flag==true)
		{
			sel_flag=false;
			obj_flag=false;
		}
	}
	
	void sel_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd)	// vel call back func.
	{
		sel_vel.linear.x=cmd->linear.x;
		sel_vel.linear.y=0;
		sel_vel.linear.z=0;
		sel_vel.angular.x=0;
		sel_vel.angular.y=0;
		sel_vel.angular.z=cmd->angular.z;
	}
	
	void obj_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd)	// vel call back func.
	{
		sel_vel.linear.x=cmd->linear.x;
		sel_vel.linear.y=0;
		sel_vel.linear.z=0;
		sel_vel.angular.x=0;
		sel_vel.angular.y=0;
		sel_vel.angular.z=cmd->angular.z;
	}
	
	void con_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd)	// vel call back func.
	{
		sel_vel.linear.x=cmd->linear.x;
		sel_vel.linear.y=0;
		sel_vel.linear.z=0;
		sel_vel.angular.x=0;
		sel_vel.angular.y=0;
		sel_vel.angular.z=-cmd->angular.z;
	}
};


///////////////////////////////////////////////////////////////////////////	main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "agricultural_robot");	// ros initialization
	sub_pub sp;									// class object delaration
	
	ros::Rate loop_rate(5);			// set 10ms loop rate
	
	while(ros::ok())				// while loop
	{
		ros::spinOnce();			// run ros once
		
		sp.mod_publish();
		
		loop_rate.sleep();			// sleep to keep the loop rate
	}
	
	return 0;					
}
