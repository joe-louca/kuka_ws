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

	double V_max = 0.1; // m/s
	double V_min = 0.00001; // m/s

	// Initialise ros node
	ros::init(argc, argv, "MyController_F");
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
		// Local force control (using delayed F and current v) - Haption input
		if(list.chk_vl && list.chk_d_fr)
		{
			// Forces
			double V_local_mag = sqrt(list.V_local[0]*list.V_local[0]+list.V_local[1]*list.V_local[1]+list.V_local[2]*list.V_local[2]);
			for (int i=0; i<3; i++)
			//double V_local_mag = sqrt(list.V_local[0]*list.V_local[0]);
			//int i = 2;
			{
				// Scale force based on the scale of v and the alignment of v against the force,
				//F_local_cmd[i] = list.delayed_F_remote[i];

				//if (V_local_mag <= V_min) {F_local_cmd[i] = 0;}
				if ((V_local_mag > V_min) && (V_local_mag <= V_max)) {F_local_cmd[i] = list.delayed_F_remote[i] * V_local_mag/V_max;}
				else if (V_local_mag >= V_max){F_local_cmd[i] = list.delayed_F_remote[i];}
				
				// Only apply a force if moving in the opposite direction?
				// if (V_local_mag <= V_min) {F_local_cmd[i] = 0;}

				// if (V_local_mag <= V_min) {F_local_cmd[i] = 0;}
				// else if ((V_local_mag > V_min) && (V_local_mag <= F_max)) {F_local_cmd[i] = list.delayed_F_remote[i] * V_local_mag/V_max;}
				// else if (V_local_mag >= V_max){F_local_cmd[i] = list.delayed_F_remote[i];}
			}
			// Repeat for torques

			// Publilsh haption force command
			F_hap_in_msg.twist.linear.x = F_local_cmd[0];
			F_hap_in_msg.twist.linear.y = F_local_cmd[1];
			F_hap_in_msg.twist.linear.z = F_local_cmd[2];
			//F_hap_in_msg.twist.angular.x = F_local_cmd[3];
			//#F_hap_in_msg.twist.angular.y = F_local_cmd[4];
			//F_hap_in_msg.twist.angular.z = F_local_cmd[5];
			F_hap_in_msg.header.stamp = list.Fr_timestamp;
			F_hap_pub.publish(F_hap_in_msg);

			// Reset checkers
			list.chk_vl = false;
			list.chk_d_fr = false;
		}

		// Spin ROS
	    ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
