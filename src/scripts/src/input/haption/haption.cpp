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


    void Q_2_YPR(float* q, float* ypr)
    {
    // Converts {x,y,z,qx,qy,qz,qw} to {x,y,z,y,p,r}
    //https://newbedev.com/is-there-an-algorithm-for-converting-quaternion-rotations-to-euler-angle-rotations
    	const double w2 = q[6]*q[6];
	const double x2 = q[3]*q[3];
	const double y2 = q[4]*q[4];
	const double z2 = q[5]*q[5];
	const double unitLength = w2 + x2 + y2 + z2;
	const double abcd = q[6]*q[4] + q[4]*q[5];
	const double eps = 1e-7;
	const double pi = 3.14159265358979323846;
	    
	if (abcd > (0.5-eps)*unitLength)
	{
		ypr[0] = q[0];
	    	ypr[1] = q[1];
	    	ypr[2] = q[2];
		ypr[3] = 2 * atan2(q[4], q[6]);		// yaw
		ypr[4] = pi;					// pitch
		ypr[5] = 0;					// roll
	}
	else if (abcd < (-0.5+eps)*unitLength)
	{
	    	ypr[0] = q[0];
	    	ypr[1] = q[1];
	    	ypr[2] = q[2];
		ypr[3] = -2 * ::atan2(q[4], q[6]);		// yaw
		ypr[4] = -pi;					// pitch
		ypr[5] = 0;					// roll
	}
	else
	{
		const double adbc = q[6]*q[5] - q[3]*q[4];
		const double acbd = q[6]*q[4] - q[3]*q[5];
		
		ypr[0] = q[0];
	    	ypr[1] = q[1];
	    	ypr[2] = q[2];
		ypr[3] = ::atan2(2*adbc, 1 - 2*(z2+x2)); 	// yaw
		ypr[4] = ::asin(2*abcd/unitLength);		// pitch
		ypr[5] = ::atan2(2*acbd, 1 - 2*(y2+x2));	// roll
	}
    }

    void Publish_Move_Cmd(float* cmd, std::string gripper_control, ros::Publisher move_pub)
    {
    // Publish cmd as twist with gripper control as the frame id
  	geometry_msgs::TwistStamped twist;
  	twist.header.stamp = ros::Time::now();
  	twist.header.frame_id = gripper_control;
  
    	twist.twist.linear.x = cmd[0];
    	twist.twist.linear.y = cmd[1];
    	twist.twist.linear.z = cmd[2];
    	twist.twist.angular.x = cmd[5];
    	twist.twist.angular.y = cmd[4];
    	twist.twist.angular.z = cmd[3];
      	move_pub.publish(twist);
    }



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "haption_stream");
    ros::NodeHandle n;
    ros::Publisher move_pub = n.advertise<geometry_msgs::TwistStamped>("hap_move_pub", 1);
    
    //n.getParam("/rate", rate); **** TO DO ****
    ros::Rate loop_rate(10);
    
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
    float * move_pos_ypr;
    
    hap_pos_q = new float[7];
    hap_pos_ypr = new float[6];
    hap_home_pos_q = new float[7];
    hap_home_pos_ypr = new float[6];
    hap_force_ypr = new float[6];
    kuka_home_pos_ypr = new float[6];
    move_pos_ypr = new float[6];
    
    double hap_factor = 1.0;
    
    hap_home_pos_q[0] = 0.3; 	//x  --- in/out 	--- LIMITS:  0.2  / 0.4
    hap_home_pos_q[1] = 0.0; 	//y  --- left/right	--- LIMITS: -0.25 / 0.25	
    hap_home_pos_q[2] = 0.07;	//z  --- down/up	--- LIMITS: -0.16 / 0.31
    hap_home_pos_q[3] = 0.0914383;	//qx
    hap_home_pos_q[4] = 0.0335527;	//qy
    hap_home_pos_q[5] = 0.0530528;	//qz
    hap_home_pos_q[6] = 0.99383;	//qw
    
    Q_2_YPR(hap_home_pos_q, hap_home_pos_ypr);
    
    //hap_home_pos_ypr[0] = 0.3;  	//x
    //hap_home_pos_ypr[1] = 0.0;  	//y
    //hap_home_pos_ypr[2] = 0.07;  	//z
    //hap_home_pos_ypr[3] = 0.101238;  	//yaw
    //hap_home_pos_ypr[4] = 0.0703094;  //pitch
    //hap_home_pos_ypr[5] = 0.0580262;  //roll    

    //n.getParam("/kuka_start_pos", kuka_home_pos_ypr); **** TO DO ****
    kuka_home_pos_ypr[0] = 505.0;  //x
    kuka_home_pos_ypr[1] = 455.0;  //y
    kuka_home_pos_ypr[2] =-896.0;  //z
    kuka_home_pos_ypr[3] = 1.249;  //yaw
    kuka_home_pos_ypr[4] =-1.007;  //pitch
    kuka_home_pos_ypr[5] = 2.868;  //roll
    
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
	    	Q_2_YPR(hap_pos_q, hap_pos_ypr);
	    	    	
	    	//d. Convert to Kuka workspace position (in rpy, not quaternions) to generate move command	
	    	for(int i = 0; i < 3; ++i) // kuka position scaled as a factor of haption
	    		{move_pos_ypr[i] = kuka_home_pos_ypr[i] + hap_factor * (hap_pos_ypr[i] - hap_home_pos_ypr[i]);}
	    	for(int i = 3; i < 6; ++i) // kuka orientation matched to haption
	    		{move_pos_ypr[i] = kuka_home_pos_ypr[i] + hap_pos_ypr[i] - hap_home_pos_ypr[i];}
	    	   	
	    	//e. Publish move command ros(THEN ADD DELAY WITH PYTHON)	
	    	Publish_Move_Cmd(move_pos_ypr, gripper_control, move_pub);
	    	   	 
	    	//f. Get delayed_ft_sensor reading
	    	
	    	
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
