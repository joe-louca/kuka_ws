#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include<ros/time.h>
#include <stdio.h>


class listener
  {
     public:
	float delayed_v_hap_out[6];
     	float delayed_F_kuka_out[6];
     	ros::Time v_timestamp;
     	ros::Time F_timestamp;
     	bool chk_dv, chk_df;
     	
  	void D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    	void D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  };

    void listener::D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_v_hap_out[0] = data->twist.linear.x;
	listener::delayed_v_hap_out[1] = data->twist.linear.y;
	listener::delayed_v_hap_out[2] = data->twist.linear.z;
	listener::delayed_v_hap_out[3] = data->twist.angular.x;
	listener::delayed_v_hap_out[4] = data->twist.angular.y;
	listener::delayed_v_hap_out[5] = data->twist.angular.z;
	listener::v_timestamp = data->header.stamp;
	listener::chk_dv = true;
    }
    
    void listener::D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_F_kuka_out[0] = data->twist.linear.x;
	listener::delayed_F_kuka_out[1] = data->twist.linear.y;
	listener::delayed_F_kuka_out[2] = data->twist.linear.z;
	listener::delayed_F_kuka_out[3] = data->twist.angular.x;
	listener::delayed_F_kuka_out[4] = data->twist.angular.y;
	listener::delayed_F_kuka_out[5] = data->twist.angular.z;
	listener::F_timestamp = data->header.stamp;
	listener::chk_df = true;
    }




int main(int argc, char* argv[])
{   
	float v_kuka_in[6] = {};
	float F_hap_in[6] = {};
	
	// Initialise ros node
    	ros::init(argc, argv, "NoController");
    	ros::NodeHandle n;	
    	
    	// Set up publisher
   	ros::Publisher v_kuka_pub = n.advertise<geometry_msgs::TwistStamped>("/v_kuka_in", 1);
   	ros::Publisher F_hap_pub = n.advertise<geometry_msgs::TwistStamped>("/F_hap_in", 1);
   	
   	// Initialise messages
   	geometry_msgs::TwistStamped v_kuka_in_msg;
   	geometry_msgs::TwistStamped F_hap_in_msg;
   	   	    	
    	// Subscribe to nodes
    	listener list;
    	ros::Subscriber d_v_hap_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_v_hap_out", 1, &listener::D_V_Callback, &list);	
    	ros::Subscriber d_F_kuka_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_F_kuka_out", 1, &listener::D_F_Callback, &list);	


	// Set the loop rate
	int rate_hz;
	n.getParam("rate_hz",rate_hz);
	ros::Rate loop_rate(rate_hz);
		    	
	while(ros::ok)
	{	
		if(list.chk_dv)
		{
			for (int i=0;i<6;i++)
			{
				// Get input input velocity command for kuka (v_kuka_in)
				v_kuka_in[i] = list.delayed_v_hap_out[i];

				// Get input force command for haption (F_hap_in)
				F_hap_in[i] = list.delayed_F_kuka_out[i];
			}
			
			// Publish kuka velocity and haption force inputs
		    	v_kuka_in_msg.twist.linear.x = v_kuka_in[0];
		    	v_kuka_in_msg.twist.linear.y = v_kuka_in[1];
		    	v_kuka_in_msg.twist.linear.z = v_kuka_in[2];
		    	v_kuka_in_msg.twist.angular.x = v_kuka_in[3];
		    	v_kuka_in_msg.twist.angular.y = v_kuka_in[4];
		    	v_kuka_in_msg.twist.angular.z = v_kuka_in[5];
		    	v_kuka_in_msg.header.stamp = list.v_timestamp;

		    	F_hap_in_msg.twist.linear.x = F_hap_in[0];
		    	F_hap_in_msg.twist.linear.y = F_hap_in[1];
		    	F_hap_in_msg.twist.linear.z = F_hap_in[2];
		    	F_hap_in_msg.twist.angular.x = F_hap_in[3];
		    	F_hap_in_msg.twist.angular.y = F_hap_in[4];
		    	F_hap_in_msg.twist.angular.z = F_hap_in[5];
		    	F_hap_in_msg.header.stamp = list.F_timestamp;
		    	
		    	v_kuka_pub.publish(v_kuka_in_msg);
		    	F_hap_pub.publish(F_hap_in_msg);

			
			// Reset checkers
			list.chk_dv = false;
			list.chk_df = false;
		}
		// Spin ROS
	    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
