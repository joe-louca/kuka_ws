#include <iostream>
#include "haption/VirtuoseAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>

#define _USE_MATH_DEFINES
 
#include <cmath>

#define HAPTION_CONTROLLER_IP "localhost" //"192.168.100.53"

    bool openConnectionToHaption(VirtContext & VC_)
    {
    	std::cout << "Connecting to Haption" << std::endl;
        VC_ = virtOpen("localhost");
        if (VC_ == NULL)
        {
            fprintf(stderr, "Error connecting: %s\n", virtGetErrorMessage(virtGetErrorCode(NULL)));
            return false;
        }
        else
        {
            std::cout << "Successfully connected" << std::endl;
            
            std::cout << "Configuring haption" << std::endl;
            // set the control model.
            // INDEXING_ALL authorizes indexing on all movements, i.e. rotations and translations.
            // INDEXING_TRANS authorizes indexing only on translations.
            // INDEXING_NONE forbids indexing on all movements
            float identity[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            virtSetIndexingMode(VC_, INDEXING_ALL);
            // set force/speed factor
            virtSetForceFactor(VC_, 1.0f);
            virtSetSpeedFactor(VC_, 1.0f);
            // set simulator intergration timestep
            virtSetTimeoutValue(VC_, 0.003f);
            // set the base reference frame with respect to the environment reference frame
            virtSetBaseFrame(VC_, identity);
            // the position of the observation reference frame with respect to the environment reference frame
            virtSetObservationFrame(VC_, identity);
            // set the command type
            virtSetCommandType(VC_, COMMAND_TYPE_IMPEDANCE);
            std::cout << "Configuration complete" << std::endl;
            
            virtSetPowerOn(VC_, 1);
            //std::cout << "Now you can activate the force-feedback at any time, by switching the power switch to ON:" << std::endl;

            return true;
        }
    }

    bool closeConnectionToHaption(VirtContext & VC_)
    {
        int result = virtClose(VC_);
        if (result != 0)
        {
            fprintf(stderr, "Error occurs when close connection to the Haption_Virtuose_Controller: %s\n", virtGetErrorMessage(virtGetErrorCode(VC_)));
            return false;
        }
        else {
            std::cout << "Successfully closed Haption connection" << std::endl;
            return true;
        }
    }

   
    bool getPosefromHaption(VirtContext & VC_, float* pose)
    {
    	std::cout << "Getting pose from Haption" << std::endl;
        int result = virtGetPosition(VC_, pose);
	std::cout << "pose:"<< pose << std::endl;
        if (result != 0)
        {
            fprintf(stderr, "Error getting pose:: %s\n", virtGetErrorMessage(virtGetErrorCode(VC_)));
            return false;
        }
        else {
            if (pose == NULL)
            {
                std::cout << "Pose is null" << std::endl;
                return false;
            }
            else 
            {
                std::cout << "Pose is: " << pose << std::endl;            
                return true;
            }
        }
    }

    void YPR_2_Q(float* ypr, float* q) // yaw: alpha (RZ), pitch: beta (RY), roll: gamma (RX)
    {
    	double yaw = ypr[3]; 	// alpha
    	double pitch = ypr[4];	// beta
    	double roll = ypr[5];	// gamma
    	
	// Set X, Y, Z
    	q[0] = ypr[0];
    	q[1] = ypr[1];
	q[2] = ypr[2];
	    	
	// Abbreviations for the various angular functions
	double cy = std::cos(yaw * 0.5);
	double sy = std::sin(yaw * 0.5);
	double cp = std::cos(pitch * 0.5);
	double sp = std::sin(pitch * 0.5);
	double cr = std::cos(roll * 0.5);
	double sr = std::sin(roll * 0.5);
	
	q[3] = sr * cp * cy - cr * sp * sy; // qx
	q[4] = cr * sp * cy + sr * cp * sy; // qy
	q[5] = cr * cp * sy - sr * sp * cy; // qz
	q[6] = cr * cp * cy + sr * sp * sy; // qw
    }

    void Q_2_YPR(float* q, float* ypr) 
    {
    	 	
    	double qx = q[3];	
    	double qy = q[4];	
    	double qz = q[5];
	double qw = q[6];
    	
    	double test = qx*qy + qz*qw;
    	double unit = qx*qx+qy*qy+qz*qz+qw*qw;

	// Set X, Y, Z
    	ypr[0] = q[0];
    	ypr[1] = q[1];
	ypr[2] = q[2];
	
    	// check for singularities
    	if (test>0.499*unit) // north pole
    	{
	    	ypr[3] = 2*std::atan2(qx,qw);
	    	ypr[4] = M_PI/2;
	    	ypr[5] = 0;
	    	return;
    	}
    	else if (test<-0.499*unit) // south pole
    	{
	    	ypr[3] = -2*std::atan2(qx,qw);
	    	ypr[4] = -M_PI/2;
	    	ypr[5] = 0;
	    	return;
    	}
    	else
    	{
    		ypr[3] = std::atan2(2*qy*qw-2*qx*qz, qx*qx-qy*qy-qz*qz+qw*qw);
    		ypr[4] = std::asin(2*test/unit);
		ypr[5] = std::atan2(2*qx*qw-2*qy*qz, -qx*qx+qy*qy-qz*qz+qw*qw);
		return;
    	}
    }


    /*void Q_2_ABG(float* q, float* ypr)
    {
    // Converts {x,y,z,qw,qx,qy,qz} to {x,y,z,alpha,beta, gamma} --> yaw(Rz)-pitch(Ry)-roll(Rx)
    //https://newbedev.com/is-there-an-algorithm-for-converting-quaternion-rotations-to-euler-angle-rotations
    	float qw = q[3];
    	float qx = q[4];
     	float qy = q[5];
     	float qz = q[6];   
    
    	float r11 = 2*(qx*qy + qw*qz);
    	float r12 = qw*qw + qx*qx - qy*qy - qz*qz;
    	float r21 =-2*(qx*qz - qw*qy);
    	float r31 = 2*(qy*qz + qw*qx);
    	float r32 = qw*qw - qx*qx - qy*qy + qz*qz;
    	
    	ypr[0] = q[0];
    	ypr[1] = q[1];
	ypr[2] = q[2];
    	ypr[3] = ::atan2(r31, r32);
    	ypr[4] = ::asin(r21);
    	ypr[5] = ::atan2(r11, r12);
    }*/

    void Publish_Move_Cmd(float* cmd, std::string gripper_control, ros::Publisher move_pub)
    {
    // Publish cmd as twist with gripper control as the frame id
  	geometry_msgs::TwistStamped twist;
  	twist.header.stamp = ros::Time::now();
  	twist.header.frame_id = gripper_control;
  
    	twist.twist.linear.x = cmd[0];	//x
    	twist.twist.linear.y = cmd[1];	//y
    	twist.twist.linear.z = cmd[2];	//z
    	twist.twist.angular.x = cmd[5];	//alpha (Rx) yaw
    	twist.twist.angular.y = cmd[4];	//beta (Ry) pitch
    	twist.twist.angular.z = cmd[3];	//gamma (Ry) roll
    	
      	move_pub.publish(twist);
    }
    
    void Publish_Move_Cmd_q(float* cmd, ros::Publisher move_pub_q)
    {
    // Publish cmd as twist with gripper control as the frame id
  	std_msgs::Float32MultiArray cmd_msg;
  	cmd_msg.data.clear(); 
    	for (int i = 0; i < 7; ++i) {cmd_msg.data.push_back(cmd[i]);}   
      	move_pub_q.publish(cmd_msg);
    }

    void Publish_FT_User(float* ft, ros::Publisher ft_user_pub)
    {
    // Publish cmd as twist with gripper control as the frame id
  	geometry_msgs::TwistStamped twist_ft;
  	twist_ft.header.stamp = ros::Time::now();
  
    	twist_ft.twist.linear.x = ft[0];	//x
    	twist_ft.twist.linear.y = ft[1];	//y
    	twist_ft.twist.linear.z = ft[2];	//z
    	twist_ft.twist.angular.x = ft[5];	//alpha (Rz) yaw
    	twist_ft.twist.angular.y = ft[4];	//beta (Ry) pitch
    	twist_ft.twist.angular.z = ft[3];	//gamma (Rx) roll
      	ft_user_pub.publish(twist_ft);
    }


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "haption_stream");
    ros::NodeHandle n;
    //ros::Publisher move_pub = n.advertise<geometry_msgs::TwistStamped>("hap_move_pub", 1);
    ros::Publisher move_pub_q = n.advertise<std_msgs::Float32MultiArray>("hap_move_pub", 1);
    ros::Publisher ft_user_pub = n.advertise<geometry_msgs::TwistStamped>("ft_user_pub", 1);
        
    int rate_hz;
    n.getParam("rate_hz",rate_hz);
    ros::Rate loop_rate(rate_hz);
    
    VirtContext VC;
    bool func_result;
    int virt_result;
    //bool btn;

    float * hap_pos_q;
    float * hap_pos_ypr;
    float * hap_home_pos_q;
    float * hap_home_pos_ypr;
    float * hap_force_ypr;
    float * kuka_home_pos_ypr;
    float * kuka_home_pos_q;
    float * move_pos_ypr;
    float * move_pos_q;
    float * ft_delay;
    float * ft_input;
    float * ft_user;
    
    hap_pos_q = new float[7];
    hap_pos_ypr = new float[6];
    hap_home_pos_q = new float[7];
    hap_home_pos_ypr = new float[6];
    hap_force_ypr = new float[6];
    kuka_home_pos_ypr = new float[6];
    kuka_home_pos_q = new float [7];
    move_pos_ypr = new float[6];
    move_pos_q = new float[7];
    ft_delay = new float[6];
    ft_input = new float[6];
    ft_user = new float[6];
    
    double hap_factor = 1000.0; // range = hap_factor * ~0.2 mm... so ~ 100cm
    double ft_factor = 0.1; // scaling factor for forces to apply to haption
    int btn_state = 0;
    int last_btn_state = 0;
    
    hap_home_pos_q[0] = 0.3; 	//x  --- in/out 	--- LIMITS:  0.2  / 0.4
    hap_home_pos_q[1] = 0.0; 	//y  --- left/right	--- LIMITS: -0.25 / 0.25	
    hap_home_pos_q[2] = 0.07;	//z  --- down/up	--- LIMITS: -0.16 / 0.31
    hap_home_pos_q[3] = 0.0914383;	//qx
    hap_home_pos_q[4] = 0.0335527;	//qy
    hap_home_pos_q[5] = 0.0530528;	//qz
    hap_home_pos_q[6] = 0.99383;	//qw
    
    Q_2_YPR(hap_home_pos_q, hap_home_pos_ypr);  

    
    
    std::vector<double> tmp;
    n.getParam("initial_kuka_pos", tmp);
    for (int i = 0; i < 6; ++i) {kuka_home_pos_ypr[i] = tmp[i];} // convert to array
    

    
    YPR_2_Q(kuka_home_pos_ypr, kuka_home_pos_q);    
    kuka_home_pos_q[3] = 0.72403;
    kuka_home_pos_q[4] = 0.64031;
    kuka_home_pos_q[5] = 0.25413;
    kuka_home_pos_q[6] = 0.03442;
    
    std::string gripper_control = "n";
    
    //1. Open connection to haption
    func_result = openConnectionToHaption(VC);
    
    if (func_result == true)
    {
	    //2. Move to home position **** TO DO ****
	    //3. Start control, enable force feedback **** TO DO ****
	    
	    //4. Loop until stop:
	    while(ros::ok())
	    {
	    	//a. Get Haption ee position - stores as hap_pos. Access elements with hap_pos[i]
	    	/* Positions are expressed as displacement vectors with seven components, i.e. one
		   translation term (x, y, z) followed by one rotation term in the form of a normalized
		   quaternion (qx, qy, qz, qw)*/
	 	virtGetPhysicalPosition(VC, hap_pos_q); 
	    	    	
	    	//b. Get haption button input
	    	/*int button_idx = 1;
	    	virtGetButton(VC, button_idx, *btn_state);
	    	
	    	if ((btn_state==1) && (last_btn_state==0)) // if button pressed then toggle gripper control
	    	{
	    		if (gripper_control = "n") {gripper_control = "y";}
	    		else if (gripper_control = "y") {gripper_control = "n";}
	    	}
	    	last_btn_state = btn_state;
	    	*/
	    	   	
		    	/*
		    	//c. Convert pose from quarternions to rpy
		    	//Q_2_YPR(hap_pos_q, hap_pos_ypr);
		    	    	
		    	//d. Convert to Kuka workspace position (in rpy, not quaternions) to generate move command	
		    	for(int i = 0; i < 3; ++i) // kuka position scaled as a factor of haption
		    		{move_pos_ypr[i] = kuka_home_pos_ypr[i] + hap_factor * (hap_pos_ypr[i] - hap_home_pos_ypr[i]);}
		    	
		    	for(int i = 3; i < 6; ++i) // kuka orientation matched to haption
		    		{move_pos_ypr[i] = kuka_home_pos_ypr[i] + 0.1*(hap_pos_ypr[i] - hap_home_pos_ypr[i]);}
				
		    	//e. Publish move command ros(THEN ADD DELAY WITH PYTHON)	
		    	Publish_Move_Cmd(move_pos_ypr, gripper_control, move_pub);
		    	*/	
	    				
	    	//d. Convert to Kuka workspace position (quaternions) to generate move command	
	    	// kuka position scaled as a factor of haption
	    	move_pos_q[0] = kuka_home_pos_q[0] + hap_factor * -(hap_pos_q[0] - hap_home_pos_q[0]); // mirror x move
	    	move_pos_q[1] = kuka_home_pos_q[1] + hap_factor * -(hap_pos_q[1] - hap_home_pos_q[1]); // mirror y move
	    	move_pos_q[2] = kuka_home_pos_q[2] + hap_factor *  (hap_pos_q[2] - hap_home_pos_q[2]); // match z move

		// kuka orientation matched to haption
	    	for(int i = 3; i < 7; ++i) {move_pos_q[i] = hap_pos_q[i];}
	    	
	    	//e. Publish move command ros(THEN ADD DELAY WITH PYTHON)
		Publish_Move_Cmd_q(move_pos_q, move_pub_q);    	
		    		
	    	
	    	//f. Get delayed_ft_sensor reading
	    	n.getParam("ft_delay/fx", ft_delay[0]);
	    	n.getParam("ft_delay/fy", ft_delay[1]);
	    	n.getParam("ft_delay/fz", ft_delay[2]);
	    	n.getParam("ft_delay/tx", ft_delay[3]);
	    	n.getParam("ft_delay/ty", ft_delay[4]);
	    	n.getParam("ft_delay/tz", ft_delay[5]);
	    	
	    	for (int i = 0; i < 6; ++i) {ft_delay[i] = ft_delay[i]*ft_factor;} // scale force readings to haption range
	    	 
	    	//g. Apply forces to haption
		virtSetForce(VC, ft_delay);
	    	
	    	//h. Get force input from the user and publish
	    	//virtGetForce(VC, ft_user);
	    	Publish_FT_User(ft_user, ft_user_pub);
	    	
	    	//h. Spin ROS
	    	ros::spinOnce();
		loop_rate.sleep();
	    	
	    }
	    
	    //5. Move back to home position **** TO DO ****
	    
	    //6. Close connection to haption
	    func_result = closeConnectionToHaption(VC);
    }  
    return 0;
}
