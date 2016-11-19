#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

using namespace std;
using namespace boost::asio;

ros::Time current_time, last_time;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

union _SPEED_
{
	unsigned char speed_buf[16];
	struct _speed_value_
	{
		float flag;
		float left_vel;
		float yspeed_vel;
		float right_vel;
	}Struct_Speed;
}Union_Speed;

geometry_msgs::Quaternion odom_quat;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;
	
	Union_Speed.Struct_Speed.flag = 15.5;
	Union_Speed.Struct_Speed.left_vel = twist_aux.linear.x;
	Union_Speed.Struct_Speed.yspeed_vel = 0.0;
	Union_Speed.Struct_Speed.right_vel = twist_aux.angular.z;
}

double dt = 0.0; 

union _Data_
{
	unsigned char buf_rev[16];
	struct _value_encoder_
	{
		float Flag_Float;
		float VX_speed;
		float VY_speed;
		float VZ_speed;
	}Struct_Encoder;
}Union_Data;
float Last_VX_speed = 0.0;
float Last_VY_speed = 0.0;
float Last_VZ_speed = 0.0;

#define VALUE_CUI 0.2
#define VALUE_MARRE 10
float Marrer_Data_L[VALUE_MARRE+1];
float Marrer_Data_R[VALUE_MARRE+1];
float Add_Left_Value,Add_Right_Value;

int main(int argc, char** argv)
{
	unsigned char check_buf[1];
	unsigned char i;
	io_service iosev;
	serial_port sp(iosev, "/dev/ttyUSB0");
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp.set_option(serial_port::parity(serial_port::parity::none));
	sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp.set_option(serial_port::character_size(8));
 
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub = n.subscribe("smoother_cmd_vel", 50, cmd_velCallback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  	tf::TransformBroadcaster odom_broadcaster;
 	
	while(ros::ok())
	{
		current_time = ros::Time::now();
		
		ros::spinOnce();  
		dt = (current_time - last_time).toSec(); 
		last_time = ros::Time::now();
		     
		current_time = ros::Time::now();
		read(sp, buffer(Union_Data.buf_rev));
		//ROS_INFO("%f   %f",Union_Speed.Struct_Speed.left_vel,Union_Speed.Struct_Speed.right_vel);
		
		if(Union_Data.Struct_Encoder.Flag_Float == 15.5)
		{
				vx = Union_Data.Struct_Encoder.VX_speed*0.796;
				vy = Union_Data.Struct_Encoder.VY_speed;
				vth = Union_Data.Struct_Encoder.VZ_speed*1.05;

				Marrer_Data_L[VALUE_MARRE]  = vx;
				Marrer_Data_R[VALUE_MARRE]  = vth;

				for(i=0;i<VALUE_MARRE;i++)
				{
					Marrer_Data_L[i]  = Marrer_Data_L[i+1];
					Marrer_Data_R[i]  = Marrer_Data_R[i+1];

					Add_Left_Value  += Marrer_Data_L[i];
					Add_Right_Value += Marrer_Data_R[i];
				}
				vx = Add_Left_Value  / VALUE_MARRE;
				vth = Add_Right_Value / VALUE_MARRE;

				Add_Left_Value  = 0;
				Add_Right_Value = 0;			

			//ROS_INFO("%f  %f   %f",vx,vy,vth);
			//ROS_INFO("connecting successed!");
		}
		else
		{	
			ROS_INFO("Fucking communication fails,The fuck can i hurry up to restart!");
			read(sp, buffer(check_buf));
		}	
		write(sp, buffer(Union_Speed.speed_buf,16)); 
		
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
		x += delta_x;
		y += delta_y;
		th += delta_th;
		
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		odom_broadcaster.sendTransform(odom_trans);
		
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
		
		odom_pub.publish(odom);
		last_time = current_time;
	}
	iosev.run(); 
}




