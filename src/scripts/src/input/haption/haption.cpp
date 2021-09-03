#include <iostream>
#include "haption/VirtuoseAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>


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

    void Q_2_ABG(float* q, float* abg)
    {
    // Converts {x,y,z,qw,qx,qy,qz} to {x,y,z,alpha,beta, gamma} --> ZYX rotation angles, or yaw-pitch-roll
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
    	
    	abg[0] = q[0];
    	abg[1] = q[1];
	abg[2] = q[2];
    	abg[3] = ::atan2(r31, r32);
    	abg[4] = ::asin(r21);
    	abg[5] = ::atan2(r11, r12);
    }

    void Publish_Move_Cmd(float* cmd, std::string gripper_control, ros::Publisher move_pub)
    {
    // Publish cmd as twist with gripper control as the frame id
  	geometry_msgs::TwistStamped twist;
  	twist.header.stamp = ros::Time::now();
  	twist.header.frame_id = gripper_control;
  
    	twist.twist.linear.x = cmd[0];	//x
    	twist.twist.linear.y = cmd[1];	//y
    	twist.twist.linear.z = cmd[2];	//z
    	twist.twist.angular.x = cmd[5];	//alpha (Rz) yaw
    	twist.twist.angular.y = cmd[4];	//beta (Ry) pitch
    	twist.twist.angular.z = cmd[3];	//gamma (Rx) roll
      	move_pub.publish(twist);
    }



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "haption_stream");
    ros::NodeHandle n;
    ros::Publisher move_pub = n.advertise<geometry_msgs::TwistStamped>("hap_move_pub", 1);
    
    int rate_hz;
    n.getParam("rate_hz",rate_hz);
    ros::Rate loop_rate(rate_hz);
    
    VirtContext VC;
    bool func_result;
    int virt_result;
    //bool btn;

    float * hap_pos_q;
    float * hap_pos_abg;
    float * hap_home_pos_q;
    float * hap_home_pos_abg;
    float * hap_force_abg;
    float * kuka_home_pos_abg;
    float * move_pos_abg;
    
    hap_pos_q = new float[7];
    hap_pos_abg = new float[6];
    hap_home_pos_q = new float[7];
    hap_home_pos_abg = new float[6];
    hap_force_abg = new float[6];
    kuka_home_pos_abg = new float[6];
    move_pos_abg = new float[6];
    
    double hap_factor = 500.0; // range = hap_factor * ~0.2 mm... so ~ 100cm
    
    hap_home_pos_q[0] = 0.3; 	//x  --- in/out 	--- LIMITS:  0.2  / 0.4
    hap_home_pos_q[1] = 0.0; 	//y  --- left/right	--- LIMITS: -0.25 / 0.25	
    hap_home_pos_q[2] = 0.07;	//z  --- down/up	--- LIMITS: -0.16 / 0.31
    hap_home_pos_q[3] = 0.0914383;	//qw
    hap_home_pos_q[4] = 0.0335527;	//qx
    hap_home_pos_q[5] = 0.0530528;	//qy
    hap_home_pos_q[6] = 0.99383;	//qz
    
    Q_2_ABG(hap_home_pos_q, hap_home_pos_abg);  

    //n.getParam("/kuka_start_pos", kuka_home_pos_ypr); **** TO DO ****
    kuka_home_pos_abg[0] = 505.0;  //x
    kuka_home_pos_abg[1] = 455.0;  //y
    kuka_home_pos_abg[2] =-896.0;  //z
    kuka_home_pos_abg[3] = 1.249;  //alpha (Rz) yaw
    kuka_home_pos_abg[4] =-1.007;  //beta (Ry) pitch
    kuka_home_pos_abg[5] = 2.868;  //gamma (Rx) roll  
    
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
	 	virtGetPhysicalPosition(VC, hap_pos_q); 
	    	    	
	    	//b. Get haption button input **** TO DO ****
	    	   	
	    	//c. Convert pose from quarternions to rpy
	    	Q_2_ABG(hap_pos_q, hap_pos_abg);
	    	    	
	    	//d. Convert to Kuka workspace position (in rpy, not quaternions) to generate move command	
	    	for(int i = 0; i < 3; ++i) // kuka position scaled as a factor of haption
	    		{move_pos_abg[i] = kuka_home_pos_abg[i] + hap_factor * (hap_pos_abg[i] - hap_home_pos_abg[i]);}
	    	
	    	for(int i = 3; i < 6; ++i) // kuka orientation matched to haption
	    		{move_pos_abg[i] = kuka_home_pos_abg[i] + hap_pos_abg[i] - hap_home_pos_abg[i];}
	    	   	
	    	//e. Publish move command ros(THEN ADD DELAY WITH PYTHON)	
	    	Publish_Move_Cmd(move_pos_abg, gripper_control, move_pub);
	    	   	 
	    	//f. Get delayed_ft_sensor reading
	    	// get param ft_delay/fx,fy,fz, tx,ty,tz
	    	
	    	//g. Apply forces to haption
	    	//hap_force[0] = 0.0; hap_force[1] = 5.0; hap_force[2] = -5.0; hap_force[3] = 0.0; hap_force[4] = 0.0; hap_force[5] = 0.0;
		//virt_result = virtAddForce(VC, hap_force);
		//if (virt_result != 0)		{fprintf(stderr, "Error adding force: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
	    	//func_result = true; // testing
	    	//hap_pos = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // testing
	    	
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
