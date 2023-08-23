#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include<ros/time.h>
#include <stdio.h>


class listener
  {
    public:
	float V_local[6];
	float F_remote[6];
	float delayed_V_local[6];
	float delayed_F_remote[6];
	ros::Time vl_timestamp;
	ros::Time d_vl_timestamp;
	ros::Time Fr_timestamp;
	ros::Time d_Fr_timestamp;
	bool chk_vl, chk_fr, chk_d_vl, chk_d_fr;

	void V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg); 	
  	void D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  };

    void listener::V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::V_local[0] = data->twist.linear.x;
	listener::V_local[1] = data->twist.linear.y;
	listener::V_local[2] = data->twist.linear.z;
	listener::V_local[3] = data->twist.angular.x;
	listener::V_local[4] = data->twist.angular.y;
	listener::V_local[5] = data->twist.angular.z;
	listener::vl_timestamp = data->header.stamp;
	listener::chk_vl = true;
    }
    
    void listener::F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::F_remote[0] = data->twist.linear.x;
	listener::F_remote[1] = data->twist.linear.y;
	listener::F_remote[2] = data->twist.linear.z;
	listener::F_remote[3] = data->twist.angular.x;
	listener::F_remote[4] = data->twist.angular.y;
	listener::F_remote[5] = data->twist.angular.z;
	listener::Fr_timestamp = data->header.stamp;
	listener::chk_fr = true;
    }

    void listener::D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_V_local[0] = data->twist.linear.x;
	listener::delayed_V_local[1] = data->twist.linear.y;
	listener::delayed_V_local[2] = data->twist.linear.z;
	listener::delayed_V_local[3] = data->twist.angular.x;
	listener::delayed_V_local[4] = data->twist.angular.y;
	listener::delayed_V_local[5] = data->twist.angular.z;
	listener::d_vl_timestamp = data->header.stamp;
	listener::chk_d_vl = true;
    }
    
    void listener::D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_F_remote[0] = data->twist.linear.x;
	listener::delayed_F_remote[1] = data->twist.linear.y;
	listener::delayed_F_remote[2] = data->twist.linear.z;
	listener::delayed_F_remote[3] = data->twist.angular.x;
	listener::delayed_F_remote[4] = data->twist.angular.y;
	listener::delayed_F_remote[5] = data->twist.angular.z;
	listener::d_Fr_timestamp = data->header.stamp;
	listener::chk_d_fr = true;
    }




int main(int argc, char* argv[])
{   
	float V_remote_cmd[6] = {};
	float F_local_cmd[6] = {};

	double F_max = 5.0; // Newtons
	double F_min = 0.2; // Newtons
	double SCALER = 5;

	// Initialise ros node
	ros::init(argc, argv, "MyController_V");
	ros::NodeHandle n;	
    	
	// Set up publisher
   	ros::Publisher v_kuka_pub = n.advertise<geometry_msgs::TwistStamped>("/v_kuka_in", 1);
   	ros::Publisher F_hap_pub = n.advertise<geometry_msgs::TwistStamped>("/F_hap_in", 1);
   	
   	// Initialise messages
   	geometry_msgs::TwistStamped v_kuka_in_msg;
   	geometry_msgs::TwistStamped F_hap_in_msg;
   	   	    	
	// Subscribe to nodes
	listener list;
	ros::Subscriber v_hap_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/v_hap_out", 1, &listener::V_Callback, &list);	
	ros::Subscriber F_kuka_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/F_kuka_out", 1, &listener::F_Callback, &list);	
	ros::Subscriber delayed_v_hap_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_v_hap_out", 1, &listener::D_V_Callback, &list);	
	ros::Subscriber delayed_F_kuka_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_F_kuka_out", 1, &listener::D_F_Callback, &list);	


	// Set the loop rate
	int rate_hz;
	n.getParam("rate_hz",rate_hz);
	ros::Rate loop_rate(rate_hz);
		    	
	while(ros::ok)
	{	
		// Remote velocity control (using delayed v and current F) - Kuka input
		if(list.chk_d_vl && list.chk_fr)
		{
			// Forces
			double F_remote_mag = sqrt(list.F_remote[0]*list.F_remote[0]+list.F_remote[1]*list.F_remote[1]+list.F_remote[2]*list.F_remote[2]);
			for (int i=0; i<3; i++)
			//double F_remote_mag = sqrt(list.F_remote[2]*list.F_remote[2]);
			//int i = 2;
			{
				if (F_remote_mag <= F_min) 
				{
					V_remote_cmd[i] = list.delayed_V_local[i];
				}
				else if ((F_remote_mag > F_min) && (F_remote_mag <= F_max)) 
				{
					V_remote_cmd[i] = list.delayed_V_local[i] / (SCALER*F_remote_mag);
				}
				else if (F_remote_mag >= F_max) 
				{
					// Check if V and F lie on the same side of the plane perpendicular to V
					// Get the equation of the perpendicluar plane that intersects the origin (a(x-x0)+b(y-y0)+c(z-z0) = 0)
					//a = vx, b = vy, c = vz
					double plane_check_V = list.delayed_V_local[0]*list.delayed_V_local[0] + list.delayed_V_local[1]*list.delayed_V_local[1] + list.delayed_V_local[2]*list.delayed_V_local[2]; 
					double plane_check_F = list.delayed_V_local[0]*list.F_remote[0] + list.delayed_V_local[1]*list.F_remote[1] + list.delayed_V_local[2]*list.F_remote[2]; 
					
					// if force and vel are in the same direction, then allow movement. If they're opposing then no movement
					if (((plane_check_V > 0) && (plane_check_F > 0)) || ((plane_check_V < 0) && (plane_check_F < 0)))
					{
						V_remote_cmd[i] = list.delayed_V_local[i] / (SCALER*F_max);
					}
					else 
					{
						V_remote_cmd[i] = 0;
					}
				}
					// // if force and vel are in the same direction, then allow movement. If they're opposing then no movement
					// if (((list.delayed_V_local[i] > 0) && (list.F_remote[i] > 0)) || ((list.delayed_V_local[i] < 0) && (list.F_remote[i] < 0)))
					// {
					// 	V_remote_cmd[i] = list.delayed_V_local[i] / (10*F_max);
					// }
					// else 
					// {
					// 	V_remote_cmd[i] = 0;
					// }
				
			}
			// Repeat for torques
			
			// Publish kuka velocity command
			v_kuka_in_msg.twist.linear.x = V_remote_cmd[0];
			v_kuka_in_msg.twist.linear.y = V_remote_cmd[1];
			v_kuka_in_msg.twist.linear.z = V_remote_cmd[2];
			//v_kuka_in_msg.twist.angular.x = V_remote_cmd[3];
			//v_kuka_in_msg.twist.angular.y = V_remote_cmd[4];
			//v_kuka_in_msg.twist.angular.z = V_remote_cmd[5];
			v_kuka_in_msg.header.stamp = list.vl_timestamp;
		    v_kuka_pub.publish(v_kuka_in_msg);

			// Reset checkers
			list.chk_vl = false;
			list.chk_fr = false;
		}

		// Spin ROS
	    ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
