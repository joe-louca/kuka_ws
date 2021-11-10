#include <iostream>
#include "haption/VirtuoseAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>

#define _USE_MATH_DEFINES
 
#include <cmath>

#define HAPTION_CONTROLLER_IP "localhost" //"192.168.100.53"

VirtContext VC;

    bool openConnectionToHaption(VirtContext & VC)
    {
    	int r;
    	std::cout << "Connecting to Haption" << std::endl;
        VC = virtOpen("localhost");
        if (VC == NULL)
        {
            fprintf(stderr, "Error connecting: %s\n", virtGetErrorMessage(virtGetErrorCode(NULL)));
            return false;
        }
        else
        {
            std::cout << "Successfully connected" << std::endl;            
            std::cout << "Configuring haption" << std::endl;
            
            float identity[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            
            r = virtSetIndexingMode(VC, INDEXING_ALL);		// Offset to move the workspace somehwere more reasonable
            	if (r != 0) {fprintf(stderr, "Error occured in SetIndexingMode: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		
            r = virtSetForceFactor(VC, 1.0f);				// set force factor
            	if (r != 0) {fprintf(stderr, "Error occured in SetForceFactor: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		
            r = virtSetSpeedFactor(VC, 1.0f);				// set speed factor
            	if (r != 0) {fprintf(stderr, "Error occured in SetSpeedFactor: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		
            r = virtSetTimeoutValue(VC, 0.1f);			// set simulator integration timestep
            	if (r != 0) {fprintf(stderr, "Error occured in SetTimeoutValue: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		
            r = virtSetBaseFrame(VC, identity);	 		// set the base reference frame with respect to the environment reference frame    
            	if (r != 0) {fprintf(stderr, "Error occured in SetBaseFrame: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		
	    r = virtSetObservationFrame(VC, identity);		// set the position of the observation reference frame with respect to the environment reference frame           
            	if (r != 0) {fprintf(stderr, "Error occured in SetObservationFrame: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		
	    r = virtSetCommandType(VC, COMMAND_TYPE_IMPEDANCE);	// set the command type
            	if (r != 0) {fprintf(stderr, "Error occured in SetCommandType: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
		            	
            std::cout << "Configuration complete" << std::endl;

            r = virtSetPowerOn(VC, 1);
            	if (r != 0) {fprintf(stderr, "Error occured in SetPowerOn: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));}
            return true;
        }
    }

    bool closeConnectionToHaption(VirtContext & VC)
    {
        int result = virtClose(VC);
        if (result != 0)
        {
            fprintf(stderr, "Error occurred in Close: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));
            return false;
        }
        else {
            std::cout << "Successfully closed Haption connection" << std::endl;
            return true;
        }
    }

   
    bool getPosefromHaption(VirtContext & VC, float* pose)
    {
    	std::cout << "Getting pose from Haption" << std::endl;
        int result = virtGetPosition(VC, pose);
	std::cout << "pose:"<< pose << std::endl;
        if (result != 0)
        {
            fprintf(stderr, "Error occurred in GetPosition: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));
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

    void YPR_2_Q(float* ypr, float* q)  // WORKING
    { // yaw: alpha (RZ), pitch: beta (RY), roll: gamma (RX) to qx,qy,qz,qw
      // https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
       
    	float yaw = ypr[3]; 	// alpha
    	float pitch = ypr[4];	// beta
    	float roll = ypr[5];	// gamma

	// Set X, Y, Z
    	q[0] = ypr[0];
    	q[1] = ypr[1];
	q[2] = ypr[2];
	    	
	// Abbreviations for the various angular functions
	float cy = std::cos(yaw * 0.5);
	float sy = std::sin(yaw * 0.5);
	float cp = std::cos(pitch * 0.5);
	float sp = std::sin(pitch * 0.5);
	float cr = std::cos(roll * 0.5);
	float sr = std::sin(roll * 0.5);
	
	q[3] = sr*cp*cy - cr*sp*sy; // qx
	q[4] = cr*sp*cy + sr*cp*sy; // qy
	q[5] = cr*cp*sy - sr*sp*cy; // qz
	q[6] = cr*cp*cy + sr*sp*sy; // qw
    }

    void Q_2_YPR(float* q, float* ypr) 
    { // https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    	float qx = q[3];
    	float qy = q[4];
     	float qz = q[5];
     	float qw = q[6];   
	float r1, r2, p1, y1, y2, yaw, pitch, roll;

	r1 = 2*(qw*qx+qy*qz);
	r2 = qw*qw - qx*qx - qy*qy + qz*qz;
	p1 = 2*(qw*qy - qx*qz);
	y1 = 2*(qw*qz+qx*qy);
	y2 = qw*qw + qx*qx - qy*qy - qz*qz;
	
	pitch = std::asin(p1);
	
  	// Gimbal lock
  	float err = 0.00001;
  	if ( (pitch > (M_PI/2)-err) && (pitch < (M_PI/2)+err) ) 
  	{
  	  	yaw = -2*atan2(qx, qw);
  		roll = 0.0;
  	}
	else if ( (pitch > (-M_PI/2)-err) && (pitch < (-M_PI/2)+err) ) 
	{
		yaw = 2*atan2(qx, qw);
		roll = 0.0;
	}
	else
	{  	
		roll = std::atan2(r1, r2);
		yaw = std::atan2(y1, y2);
	}
	
    	ypr[0] = q[0];				// X
    	ypr[1] = q[1];				// Y
	ypr[2] = q[2];				// Z
    	ypr[3] = yaw;				//rz
	ypr[4] = pitch;			//ry
    	ypr[5] = roll; 			//rx 	
    }


    void Publish_Move_Cmd_ypr(float* cmd_ypr, ros::Publisher move_pub_ypr)
    {
    // Publish cmd as Float32MultiArray
  	std_msgs::Float32MultiArray cmd_msg_ypr;
  	cmd_msg_ypr.data.clear(); 
    	for (int i = 0; i < 6; ++i) {cmd_msg_ypr.data.push_back(cmd_ypr[i]);}   
    	std::cout<<"publishing ypr"<<std::endl;
      	move_pub_ypr.publish(cmd_msg_ypr);
    }
    
    void Publish_Move_Cmd_q(float* cmd, ros::Publisher move_pub_q)
    {
    // Publish cmd as Float32MultiArray
  	std_msgs::Float32MultiArray cmd_msg;
  	cmd_msg.data.clear(); 
    	for (int i = 0; i < 7; ++i) {cmd_msg.data.push_back(cmd[i]);}   
      	move_pub_q.publish(cmd_msg);
    }    
    
    /*void Publish_Move_Cmd_rot(float* cmd, ros::Publisher move_pub)
    {
    // Publish cmd as twist with gripper control as the frame id
  	std_msgs::Float32MultiArray cmd_msg;
  	cmd_msg.data.clear(); 
    	for (int i = 0; i < 12; ++i) {cmd_msg.data.push_back(cmd[i]);}   
      	move_pub.publish(cmd_msg);
    }
    */

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

    void FTCallback(const geometry_msgs::WrenchStamped& wrench_msg)
    {  
    	std::cout<<"in callback"<<std::endl;
    	float ft_factor = 0.1;
    	float * ft_delay;	
    	ft_delay = new float[6];
    	ft_delay[0] = wrench_msg.wrench.force.x;
    	ft_delay[1] = wrench_msg.wrench.force.y;
    	ft_delay[2] = wrench_msg.wrench.force.z;    	
    	ft_delay[3] = wrench_msg.wrench.torque.x;   	
    	ft_delay[4] = wrench_msg.wrench.torque.y;   	
    	ft_delay[5] = wrench_msg.wrench.torque.z;
    	    	
    	for (int i = 0; i < 6; ++i) {ft_delay[i] = ft_delay[i]*ft_factor; // scale force readings to haption range
    	std::cout<< ft_delay[i]<<", ";}
    	std::cout<<std::endl;
    	
    	//virtSetForce(VC, ft_delay);
    }

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "haption_stream");
    ros::NodeHandle n;
    ros::Publisher move_pub_q = n.advertise<std_msgs::Float32MultiArray>("hap_move_pub_q", 1);
    //ros::Publisher move_pub_ypr = n.advertise<geometry_msgs::TwistStamped>("hap_move_pub_ypr", 1);
    ros::Publisher ft_user_pub = n.advertise<geometry_msgs::TwistStamped>("ft_user_pub", 1);
    ros::Subscriber ft_sub;
        
    int rate_hz;
    n.getParam("rate_hz",rate_hz);
    rate_hz = 1000;
    ros::Rate loop_rate(rate_hz);
        
    bool func_result;
    int virt_result;

    float hap_pos_q[7] = {};
    float hap_home_pos_q[7] = {};   
    float kuka_home_pos_ypr[6] = {};
    float kuka_home_pos_q[7] = {};

    float move_pos_ypr[6] = {};
    float move_pos_q[7] = {};

    float hap_force_ypr[6] = {};
    float ft_delay[6] = {};
    float ft_input[6] = {};
    float ft_user[6] = {};

    // Initialise button and gripper control variables    
    int button_states[4] = {0,0,0,0};
    int btn_state = 0;
    int last_btn_state = 0;
    int force_fb_btn;
    std::string gripper_control = "n";
    
    // Set Haption home position values
    hap_home_pos_q[0] = 0.3; 	//x  --- in/out 	--- LIMITS:  0.2  / 0.4
    hap_home_pos_q[1] = 0.0; 	//y  --- left/right	--- LIMITS: -0.25 / 0.25	
    hap_home_pos_q[2] = 0.07;	//z  --- down/up	--- LIMITS: -0.16 / 0.31
    hap_home_pos_q[3] = 0.0;//0.0914383;	//qx
    hap_home_pos_q[4] = 0.0;//0.0335527;	//qy
    hap_home_pos_q[5] = 0.0;//0.0530528;	//qz
    hap_home_pos_q[6] = 1.0;//0.99383;	//qw
    
    // Get Kuka Home position values in YPR and Quats
    std::vector<double> tmp;
    n.getParam("initial_kuka_pos", tmp);
    for (int i = 0; i < 6; ++i) {kuka_home_pos_ypr[i] = tmp[i];} // convert to array
    YPR_2_Q(kuka_home_pos_ypr, kuka_home_pos_q);

     
    double ws_factor = 1000.0; // range = ws_factor * ~0.2 mm... so ~ 200cm?
    double ft_factor = 0.01; // scaling factor for forces to apply to haption
    
    //1. Open connection to haption
    func_result = openConnectionToHaption(VC);
   
   
    //2. Make sure force feedback button is not pressed to start
    for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &button_states[i]);}
    force_fb_btn = button_states[2];
    bool force_fb_checker = false;
    std::cout<<"Force feedback button must be un-pressed to start"<<std::endl;
    while(!force_fb_checker)
    {
    	if(force_fb_btn==0) {force_fb_checker = true;}
    	else
    	{
    	    for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &button_states[i]);}
    	    force_fb_btn = button_states[2];
	}
    }
    std::cout<<"Starting control"<<std::endl;
    
    if (func_result == true)
    {
	    
	    //4. Loop until stop:
	    while(ros::ok())
	    {
	    	//a. Get Haption ee position - stores as hap_pos. Access elements with hap_pos[i]
	    	//(x, y, z, qx, qy, qz, qw)
	 	virt_result = virtGetPosition(VC, hap_pos_q); 		   

	    	 	    	
	    	//b. Get haption buttons input
	    	for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &button_states[i]);}
		// button_states[i]==0 or 1, where 0 = left, 1 = mid, 2 = force fb, right = ?
		
		force_fb_btn = button_states[2];
		btn_state = button_states[1];
	    	if ((btn_state==1) && (last_btn_state==0)) // if middle button pressed then toggle gripper control
	    	{
	    		if (gripper_control == "n") {gripper_control = "y";}
	    		else if (gripper_control == "y") {gripper_control = "n";}
	    	}
	    	last_btn_state = btn_state;
	    	
	    				
	    	//d. Convert to Kuka workspace position (quaternions) to generate move command	
	    	// kuka position scaled as a factor of haption
	    	move_pos_q[0] = kuka_home_pos_q[0] + ws_factor * (hap_pos_q[0] - hap_home_pos_q[0]); // x 
	    	move_pos_q[1] = kuka_home_pos_q[1] + ws_factor * (hap_pos_q[1] - hap_home_pos_q[1]); // y 
	    	move_pos_q[2] = kuka_home_pos_q[2] + ws_factor * (hap_pos_q[2] - hap_home_pos_q[2]); // z    
	    	 	
		// kuka orientation matched to haption
	    	for(int i = 3; i < 7; ++i) {move_pos_q[i] = hap_pos_q[i];}
	    		    	
	    	//e. Publish move command ros(THEN ADD DELAY WITH PYTHON)
		Publish_Move_Cmd_q(move_pos_q, move_pub_q);
		
	 	Q_2_YPR(move_pos_q, move_pos_ypr); // convert to YPR for real KUKA
	 	//std::cout<<"A: "<<move_pos_ypr[3]<<", B: "<<move_pos_ypr[4]<<", C: "<<move_pos_ypr[5]<<std::endl;
	 	//Publish_Move_Cmd_ypr(move_pos_ypr, move_pub_ypr);
	 	n.setParam("hap_move_pub_ypr/x", move_pos_ypr[0]);	 	
	 	n.setParam("hap_move_pub_ypr/y", move_pos_ypr[1]);
	 	n.setParam("hap_move_pub_ypr/z", move_pos_ypr[2]);
	 	n.setParam("hap_move_pub_ypr/a", move_pos_ypr[3]);
	 	n.setParam("hap_move_pub_ypr/b", move_pos_ypr[4]);
	 	n.setParam("hap_move_pub_ypr/c", move_pos_ypr[5]);
		    		
		if (force_fb_btn==1)
		{	
		    	//f. Get delayed_ft_sensor reading & apply forces to haption
		    	n.getParam("ft_delay/fx",ft_delay[0]);
		    	n.getParam("ft_delay/fy",ft_delay[1]);
		    	n.getParam("ft_delay/fz",ft_delay[2]);	    	
		    	n.getParam("ft_delay/tx",ft_delay[3]);
		    	n.getParam("ft_delay/ty",ft_delay[4]);
		    	n.getParam("ft_delay/tz",ft_delay[5]);
						
			// Scale and Reverse force direction for x and z (copsim only)
			ft_delay[0] = -ft_delay[0]*ft_factor;
			ft_delay[1] = ft_delay[1]*ft_factor;
			ft_delay[2] = -ft_delay[2]*ft_factor;
			ft_delay[3] = 0.0f;
			ft_delay[4] = 0.0f;
			ft_delay[5] = 0.0f;
			
			// get forces within range
			//FX,FY,FZ must be in range -15.0-15.0f, TX,TY,TZ must be in range -1.0-1.0f	
			for(int i=0; i<3; i++)
			{
				if(ft_delay[i]>3.0f) {ft_delay[i]=3.0f;}
				if(ft_delay[i]<-3.0f) {ft_delay[i]=-3.0f;}
				if(ft_delay[i+3]>0.3f) {ft_delay[i+3]=0.3f;}
				if(ft_delay[i+3]<-0.3f){ft_delay[i+3]=-0.3f;}
			}
			
			
		    	//g. Apply forces to haption
			virt_result = virtSetForce(VC, ft_delay);		// Only works in COMMAND_TYPE_IMPEDANCE - doesnt work in COMMAND_TYPE_VIRTMECH
			if (virt_result != 0) {fprintf(stderr, "Error in SetForce: %s\n",virtGetErrorMessage(virtGetErrorCode(VC)));}

			
		    	//h. Get force input from the user and publish
		    	//virt_result = virtGetForce(VC, ft_user); // Only works in COMMAND_TYPE_VIRTMECH - doesnt work in COMMAND_TYPE_IMPEDANCE
		    	//if (virt_result == -1) {fprintf(stderr, "Error in GetForce: %s\n",virtGetErrorMessage(virtGetErrorCode(VC)));}
			//else {std::cout<<"GET: x: "<<ft_user[0]<<", y: "<<ft_user[1]<<", z: "<<ft_user[2]<<", tx: "<<ft_user[3]<<", ty: "<<ft_user[4]<<", tz: "<<ft_user[5]<<std::endl;}
		

	    		//Publish_FT_User(ft_user, ft_user_pub);
	    	}
	    	//h. Spin ROS
	    	ros::spinOnce();
		loop_rate.sleep();
	    	
	    }
	    
	    //5. Move back to home position **** TO DO ****
	    
	    // Other virts to try
	    // virtGetSpeed(VC, speed)
	    // virtGetDeadMan(VC, &deadman)
	    // virtSetPosition(VC, pos);
	    // virtSetSpeed(VC, speed);
	    
	    
	    //6. Close connection to haption
	    func_result = closeConnectionToHaption(VC);
    } 
     
    return 0;
}
