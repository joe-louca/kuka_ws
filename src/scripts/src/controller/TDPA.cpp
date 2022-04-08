#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include<ros/time.h>
#include <stdio.h>


class listener
  {
     public:
     	double timestep;
     	double delayed_timestep;
     	float v_hap_out[6];
	float delayed_v_hap_out[6];
	float delayed_delayed_v_hap_out[6];
     	float F_kuka_out[6];
     	float delayed_F_kuka_out[6];
     	float delayed_delayed_F_kuka_out[6];
     	bool chk_v, chk_dv, chk_ddv, chk_f, chk_df, chk_ddf, chk_t, chk_dt;
     	ros::Time v_timestamp;
     	ros::Time F_timestamp;
     	
     	void T_Callback(const std_msgs::Float64::ConstPtr& msg);
     	void D_T_Callback(const std_msgs::Float64::ConstPtr& msg);
     	void V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
     	void D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
     	void D_D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);     	
     	void F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
     	void D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
     	void D_D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
   };
   
    void listener::T_Callback(const std_msgs::Float64::ConstPtr& msg)
    {
	listener::timestep = msg->data;
	listener::chk_t = true;
    }
    
    void listener::D_T_Callback(const std_msgs::Float64::ConstPtr& msg)
    {
	listener::delayed_timestep = msg->data;
	listener::chk_dt = true;
    }    
    
    void listener::V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::v_hap_out[0] = data->twist.linear.x;
	listener::v_hap_out[1] = data->twist.linear.y;
	listener::v_hap_out[2] = data->twist.linear.z;
	listener::v_hap_out[3] = data->twist.angular.x;
	listener::v_hap_out[4] = data->twist.angular.y;
	listener::v_hap_out[5] = data->twist.angular.z;
	listener::v_timestamp = data->header.stamp;
	listener::chk_v = true;
    }
    
    void listener::D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_v_hap_out[0] = data->twist.linear.x;
	listener::delayed_v_hap_out[1] = data->twist.linear.y;
	listener::delayed_v_hap_out[2] = data->twist.linear.z;
	listener::delayed_v_hap_out[3] = data->twist.angular.x;
	listener::delayed_v_hap_out[4] = data->twist.angular.y;
	listener::delayed_v_hap_out[5] = data->twist.angular.z;
	listener::chk_dv = true;
    }
    
    void listener::D_D_V_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_delayed_v_hap_out[0] = data->twist.linear.x;
	listener::delayed_delayed_v_hap_out[1] = data->twist.linear.y;
	listener::delayed_delayed_v_hap_out[2] = data->twist.linear.z;
	listener::delayed_delayed_v_hap_out[3] = data->twist.angular.x;
	listener::delayed_delayed_v_hap_out[4] = data->twist.angular.y;
	listener::delayed_delayed_v_hap_out[5] = data->twist.angular.z;
	listener::chk_ddv = true;
    }

    void listener::F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::F_kuka_out[0] = data->twist.linear.x;
	listener::F_kuka_out[1] = data->twist.linear.y;
	listener::F_kuka_out[2] = data->twist.linear.z;
	listener::F_kuka_out[3] = data->twist.angular.x;
	listener::F_kuka_out[4] = data->twist.angular.y;
	listener::F_kuka_out[5] = data->twist.angular.z;
	listener::F_timestamp = data->header.stamp;
	listener::chk_f = true;
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
	listener::chk_df = true;
    }
    
    void listener::D_D_F_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
	const geometry_msgs::TwistStamped* data = msg.get();
	listener::delayed_delayed_F_kuka_out[0] = data->twist.linear.x;
	listener::delayed_delayed_F_kuka_out[1] = data->twist.linear.y;
	listener::delayed_delayed_F_kuka_out[2] = data->twist.linear.z;
	listener::delayed_delayed_F_kuka_out[3] = data->twist.angular.x;
	listener::delayed_delayed_F_kuka_out[4] = data->twist.angular.y;
	listener::delayed_delayed_F_kuka_out[5] = data->twist.angular.z;
	listener::chk_ddf = true;
    }



int main(int argc, char* argv[])
{   
	double sampling_time = 0;
	double prev_timestep;
	double prev_delayed_timestep;

	float P_hap_out[6] = {};
	float P_kuka_out[6] = {};
	float delayed_P_hap_in[6] = {};
	float delayed_P_kuka_in[6] = {};
	float E_hap_out[6] = {};
	float E_kuka_out[6] = {};
	float delayed_E_hap_in[6] = {};
	float delayed_E_kuka_in[6] = {};
	float W_A[6] = {};
	float W_B[6] = {};
	float E_A_total[6] = {};
	float E_B_total[6] = {};
	float alpha[6] = {};
	float beta[6] = {};
	float v_kuka_in[6] = {};
	float F_hap_in[6] = {};
	
	// Initialise ros node
    	ros::init(argc, argv, "TDPA");
    	ros::NodeHandle n;	
    	
    	// Set up publisher
   	ros::Publisher v_kuka_pub = n.advertise<geometry_msgs::TwistStamped>("/v_kuka_in", 1);
   	ros::Publisher F_hap_pub = n.advertise<geometry_msgs::TwistStamped>("/F_hap_in", 1);
   	
   	// Initialise messages
   	geometry_msgs::TwistStamped v_kuka_in_msg;
   	geometry_msgs::TwistStamped F_hap_in_msg;
   	   	    	
    	// Subscribe to nodes
    	listener list;
    	ros::Subscriber timestep_sub = n.subscribe<std_msgs::Float64>("/timestep", 1, &listener::T_Callback, &list);	
    	ros::Subscriber d_timestepsub = n.subscribe<std_msgs::Float64>("/delayed_timestep", 1, &listener::D_T_Callback, &list);	    	
    	ros::Subscriber v_hap_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/v_hap_out", 1, &listener::V_Callback, &list);	
    	ros::Subscriber d_v_hap_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_v_hap_out", 1, &listener::D_V_Callback, &list);	
    	ros::Subscriber d_d_v_hap_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_delayed_v_hap_out", 1, &listener::D_D_V_Callback, &list);	
    	ros::Subscriber F_kuka_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/F_kuka_out", 1, &listener::F_Callback, &list);	
    	ros::Subscriber d_F_kuka_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_F_kuka_out", 1, &listener::D_F_Callback, &list);	
    	ros::Subscriber d_d_F_kuka_out_sub = n.subscribe<geometry_msgs::TwistStamped>("/delayed_delayed_F_kuka_out", 1, &listener::D_D_F_Callback, &list);

	// Set the loop rate
	int rate_hz;
	n.getParam("rate_hz",rate_hz);
	ros::Rate loop_rate(rate_hz);
		    	
	while(ros::ok)
	{	
		//printf("\n T: %s", list.chk_t ? "true" : "false");
		if(list.chk_ddv)
		{
			sampling_time += list.timestep;
			printf("\n\n LOOP"); 
			for (int i=0;i<6;i++)
			{
				// Calculate power out at haption side 
				P_hap_out[i] = -(list.delayed_F_kuka_out[i] * list.v_hap_out[i]);
				if (P_hap_out[i] > 0) {P_hap_out[i] = 0;}

				// Calculate power out at kuka side
				P_kuka_out[i] = -(list.F_kuka_out[i] * list.delayed_v_hap_out[i]);
				if (P_kuka_out[i] > 0) {P_kuka_out[i] = 0;}

				// Calculate delayed power in at haption side
				delayed_P_hap_in[i] = list.delayed_delayed_F_kuka_out[i] * list.delayed_v_hap_out[i];
				if (delayed_P_hap_in[i] < 0) {delayed_P_hap_in[i] = 0;}
					
				// Calculate delayed power in at kuka side
				delayed_P_kuka_in[i] = list.delayed_F_kuka_out[i] * list.delayed_delayed_v_hap_out[i];
				if (delayed_P_kuka_in[i] < 0) {delayed_P_kuka_in[i] = 0;}

				// Calculate energies in and out of haption and kuka sides
				E_hap_out[i] = P_hap_out[i] * (list.timestep - prev_timestep);
				E_kuka_out[i] = P_kuka_out[i] * (list.timestep - prev_timestep);
				delayed_E_hap_in[i] = delayed_P_hap_in[i] * (list.delayed_timestep - prev_delayed_timestep);
				delayed_E_kuka_in[i] = delayed_P_kuka_in[i] * (list.delayed_timestep - prev_delayed_timestep);
				
				// Calculate energy to be dissipated by passivity controllers A & B (W_A, W_B)
				W_A[i] = delayed_E_hap_in[i] - E_kuka_out[i] - E_A_total[i];
				W_B[i] = delayed_E_kuka_in[i] - E_hap_out[i] - E_B_total[i];

				// Update total energy dissipated by passivity controllers A & B so far
				E_A_total[i] += W_A[i];
				E_B_total[i] += W_B[i];
				
				// Calculate variable damping of PC A (alpha)
				if (W_A[i] < 0)
					{alpha[i] = -W_A[i] / (sampling_time * list.F_kuka_out[i]*list.F_kuka_out[i]);}
				else
					{alpha[i] = 0;}

				// Calculate variable damping of PC B (beta)
				if (W_B < 0)
					{beta[i] = -W_B[i] / (sampling_time * list.v_hap_out[i]*list.v_hap_out[i]);}
				else
					{beta[i] = 0;}

				// Calculate PC A controlled input velocity command for kuka (v_kuka_in)
				v_kuka_in[i] = list.delayed_v_hap_out[i] - alpha[i] * list.F_kuka_out[i];
				printf("\nv_kuka_in %i = %f", i, v_kuka_in[i]); 

				// Calculate PC B controlled input force command for haption (F_hap_in)
				F_hap_in[i] = list.delayed_F_kuka_out[i] - beta[i] * list.v_hap_out[i];
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
		    	
			// Update timesteps
			prev_timestep = list.timestep;
			prev_delayed_timestep = list.delayed_timestep;
			
			// Reset checkers
			list.chk_t = false;
			list.chk_dt = false;
			list.chk_v = false;
			list.chk_dv = false;
			list.chk_ddv = false;
			list.chk_f = false;
			list.chk_df = false;
			list.chk_ddf = false;
		}
		// Spin ROS
	    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
