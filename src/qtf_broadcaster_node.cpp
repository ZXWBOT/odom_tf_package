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

#define width_robot 0.33
#define ticks_per_meter 19660.8


ros::Time current_time, last_time;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
unsigned char buf[6] = {0xff,0x00,0x00,0x00,0x00,0xfe};

union _SPEED_
{
	unsigned char speed_buf[12];
	struct _speed_value_
	{
		unsigned int flag;
		float right_vel;
		float left_vel;
	}Struct_Speed;
}Union_Speed;

/*******************************timr init******************/ 

geometry_msgs::Quaternion odom_quat;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	if(vel_x == 0)
	{
		Union_Speed.Struct_Speed.right_vel = vel_th * width_robot / 2.0;
		Union_Speed.Struct_Speed.left_vel = (-1) * Union_Speed.Struct_Speed.right_vel;
	}
	else if(vel_th == 0)
	{
		Union_Speed.Struct_Speed.left_vel = Union_Speed.Struct_Speed.right_vel = vel_x;
	}
	else
	{
		Union_Speed.Struct_Speed.left_vel = vel_x - vel_th * width_robot / 2.0;
		Union_Speed.Struct_Speed.right_vel = vel_x + vel_th * width_robot / 2.0;
	}
}

struct cul_encoder
{
	float Encoder_Old_Right;
	float Encoder_Old_Left;
	float Distance_Right;
	float Distance_Left;
	float distance_xy;
	float distance_th;
}Cul_distance_Speed;
double dt = 0.0;
unsigned char Usart_Recive_Buff[4];  

unsigned char Serial_Rx_Sta = 0;
unsigned char Flag_Receivce = 0;

union _Data_
{
	unsigned char buf_rev[16];
	struct _value_encoder_
	{
		float Flag_Float;
		signed int Right_Encoder;
		signed int Left_Encoder;
		signed int Mangnetometer;
	}Struct_Encoder;
}Union_Data;

#define VALUE_MARRE  20
float Add_Left_Value;
float Marrer_Data_L[VALUE_MARRE+1];
float Add_Right_Value;
float Marrer_Data_R[VALUE_MARRE+1];
float Add_Dt_Time;
float Marrer_Data_Dt[VALUE_MARRE+1];

int main(int argc, char** argv)
{
	unsigned char i = 0;
	unsigned char check_buf[1];
	io_service iosev;
	serial_port sp(iosev, "/dev/ttyUSB0");
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp.set_option(serial_port::parity(serial_port::parity::none));
	sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp.set_option(serial_port::character_size(8));
 
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 50, cmd_velCallback);
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

		if(Union_Data.Struct_Encoder.Flag_Float == 1.5)
		{
			if(Union_Data.Struct_Encoder.Right_Encoder == 0.0)
			{
				Cul_distance_Speed.Distance_Right = 0.0;
				Cul_distance_Speed.Distance_Left  = 0.0;
			}
			else
			{   
				Cul_distance_Speed.Distance_Right = -(Union_Data.Struct_Encoder.Right_Encoder - Cul_distance_Speed.Encoder_Old_Right) / ticks_per_meter;
				Cul_distance_Speed.Distance_Left  = (Union_Data.Struct_Encoder.Left_Encoder - Cul_distance_Speed.Encoder_Old_Left) / ticks_per_meter;
				/*******************滑动数组滤波*********************/
				Marrer_Data_L[VALUE_MARRE]  = Cul_distance_Speed.Distance_Left;
				Marrer_Data_R[VALUE_MARRE]  = Cul_distance_Speed.Distance_Right;
				Marrer_Data_Dt[VALUE_MARRE] = dt;

				for(i=0;i<VALUE_MARRE;i++)
				{
				Marrer_Data_L[i]  = Marrer_Data_L[i+1];
				Marrer_Data_R[i]  = Marrer_Data_R[i+1];
				Marrer_Data_Dt[i] = Marrer_Data_Dt[i+1];

				Add_Left_Value  += Marrer_Data_L[i];
				Add_Right_Value += Marrer_Data_R[i];
				Add_Dt_Time     += Marrer_Data_Dt[i];
				}
				Cul_distance_Speed.Distance_Left  = Add_Left_Value  / VALUE_MARRE;
				Cul_distance_Speed.Distance_Right = Add_Right_Value / VALUE_MARRE;
				dt = Add_Dt_Time / VALUE_MARRE;

				Add_Left_Value  = 0;
				Add_Right_Value = 0;
				Add_Dt_Time     = 0;

				/***********************************************/			
			}

	 		Cul_distance_Speed.Encoder_Old_Right = (float)Union_Data.Struct_Encoder.Right_Encoder;
			Cul_distance_Speed.Encoder_Old_Left  = (float)Union_Data.Struct_Encoder.Left_Encoder;

			Cul_distance_Speed.distance_xy = (Cul_distance_Speed.Distance_Right + Cul_distance_Speed.Distance_Left) / 2.0;
			Cul_distance_Speed.distance_th = (Cul_distance_Speed.Distance_Right - Cul_distance_Speed.Distance_Left) / width_robot;

				
			vx  = Cul_distance_Speed.distance_xy / dt;
			vth = Cul_distance_Speed.distance_th / dt;
			ROS_INFO("Hello ZXBOT! ^~^ Very Glad to chat with you!");
			Union_Speed.Struct_Speed.flag = 0xff;
		}
		else
		{	
			ROS_INFO("Fucking communication fails,The fuck can i hurry up to restart!");
			read(sp, buffer(check_buf));
			Union_Speed.Struct_Speed.flag = 0xfe;
		}	

		write(sp, buffer(Union_Speed.speed_buf,12)); 

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



