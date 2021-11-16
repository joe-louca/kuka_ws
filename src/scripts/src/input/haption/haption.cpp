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

    void Q_Add(float* QA, float* QB, float* QC)
    { // Adds XYZ translation B to A, and adds Q rotation B to A to give QC (multiply quaternions)
    	float QAx = QA[0];
    	float QAy = QA[1];
    	float QAz = QA[2];
    	float QAqx = QA[3];
    	float QAqy = QA[4];
    	float QAqz = QA[5];
    	float QAqw = QA[6];
    	
      	float QBx = QB[0];
    	float QBy = QB[1];
    	float QBz = QB[2];
    	float QBqx = QB[3];
    	float QBqy = QB[4];
    	float QBqz = QB[5];
    	float QBqw = QB[6];
    	
    	/*Q_2_Rot(QA, RotA);
    	Q_2_Rot(QB, RotB);
    	RotC = Rot_Multiply(RotB, RotA) // Rot A first, then Rot B around A 
    	Rot_2_Q(RotC, QC)
    	QC[0] = QAx + QBx;	// X
    	QC[1] = QAy + QBy;	// Y
    	QC[2] = QAz + QBz;	// Z      	
    	*/
    	
    	QC[0] = QAx + QBx;	// X
    	QC[1] = QAy + QBy;	// Y
    	QC[2] = QAz + QBz;	// Z    		
    	QC[3] = QAqw * QBqw - QAqx * QBqx - QAqy * QBqy - QAqz * QBqz; // qw
    	QC[4] = QAqw * QBqx + QAqx * QBqw + QAqy * QBqz - QAqz * QBqy; // qx
    	QC[5] = QAqw * QBqy - QAqx * QBqz + QAqy * QBqw + QAqz * QBqx; // qy
    	QC[6] = QAqw * QBqz + QAqx * QBqy - QAqy * QBqx + QAqz * QBqw; // qz  	
    } 

    void Q_A2B(float* QA, float* QB, float* QC)
    { // QC is the rotation from QA -> QB
    	float QAx = QA[0];
    	float QAy = QA[1];
    	float QAz = QA[2];
    	// Inverse Q = conjugate Q = -qx -qy -qz +qw / Norm
    	float QAqx = -1*QA[3];
    	float QAqy = -1*QA[4];
    	float QAqz = -1*QA[5];
    	float QAqw =  1*QA[6];
    	
      	float QBx = QB[0];
    	float QBy = QB[1];
    	float QBz = QB[2];
    	float QBqx = QB[3];
    	float QBqy = QB[4];
    	float QBqz = QB[5];
    	float QBqw = QB[6];
    	
    	// Find X Y Z difference
    	QC[0] = QAx - QBx;	// X
    	QC[1] = QAy - QBy;	// Y
    	QC[2] = QAz - QBz;	// Z
    	
    	//Find rotation: QC = QA_inverse * QB
    	QC[3] = QAqw * QBqw - QAqx * QBqx - QAqy * QBqy - QAqz * QBqz; // qw
    	QC[4] = QAqw * QBqx + QAqx * QBqw + QAqy * QBqz - QAqz * QBqy; // qx
    	QC[5] = QAqw * QBqy - QAqx * QBqz + QAqy * QBqw + QAqz * QBqx; // qy
    	QC[6] = QAqw * QBqz + QAqx * QBqy - QAqy * QBqx + QAqz * QBqw; // qz     	

    }


    
    void Publish_Move_Cmd_q(float* cmd, ros::Publisher move_pub_q)
    {  // Publish cmd as Float32MultiArray
  	std_msgs::Float32MultiArray cmd_msg;
  	cmd_msg.data.clear(); 
    	for (int i = 0; i < 7; ++i) {cmd_msg.data.push_back(cmd[i]);}   
      	move_pub_q.publish(cmd_msg);
    }    

/*

    void Publish_Move_Cmd_ypr(float* cmd_ypr, ros::Publisher move_pub_ypr)
    {  // Publish cmd as Float32MultiArray
  	std_msgs::Float32MultiArray cmd_msg_ypr;
  	cmd_msg_ypr.data.clear(); 
    	for (int i = 0; i < 6; ++i) {cmd_msg_ypr.data.push_back(cmd_ypr[i]);}   
      	move_pub_ypr.publish(cmd_msg_ypr);
    }
    
    void FTCallback(const geometry_msgs::WrenchStamped& wrench_msg)
    {  
    	std::cout<<"in callback"<<std::endl;
    	float ft_factor = 0.1;
    	float* ft_delay[6] = {};
    	ft_delay[0] = wrench_msg.wrench.force.x;
    	ft_delay[1] = wrench_msg.wrench.force.y;
    	ft_delay[2] = wrench_msg.wrench.force.z;    	
    	ft_delay[3] = wrench_msg.wrench.torque.x;   	
    	ft_delay[4] = wrench_msg.wrench.torque.y;   	
    	ft_delay[5] = wrench_msg.wrench.torque.z;    	    	
    	std::cout<< ft_delay[i]<<", ";
    	std::cout<<std::endl;
    	
    	//virtSetForce(VC, ft_delay);
    }
*/
    
int main(int argc, char* argv[])
{

    ros::init(argc, argv, "haption_stream");
    ros::NodeHandle n;
    ros::Publisher move_pub_q = n.advertise<std_msgs::Float32MultiArray>("hap_move_pub_q", 1);
    //ros::Subscriber delayed_ft_sub = n.subscribe("delayed_ft", 1, FTCallback);					// FIX THIS 
      
    int rate_hz;
    n.getParam("rate_hz",rate_hz);
    //rate_hz = 1000;
    ros::Rate loop_rate(rate_hz);
    
    // Set some scale factors
    double ws_factor = 500.0; // kuka_range = ws_factor * haption_range?
    double ft_factor = 0.01; // scaling factor for forces to apply to haption
     
    bool func_result;
    int virt_result;

    // Declare some position arrays
    float hap_pos_q[7] = {};
    float hap_pos_ypr[6] = {};
    float hap_start_pos_q[7] = {};   
    float hap_start_pos_ypr[6] = {};   
    float hap_move_q[7] = {};
    float hap_move_ypr[6] = {};
    
    float kuka_start_pos_ypr[6] = {};
    float kuka_start_pos_q[7] = {};

    float move_pos_ypr[6] = {};
    float move_pos_q[7] = {};

    float hap_force_ypr[6] = {};
    float ft_delay[6] = {};
    float ft_input[6] = {};
    float ft_user[6] = {};

    // Declare button and gripper control variables    
    int buttons[4] = {0,0,0,0};
    int left_btn = 0;
    int mid_btn = 0;
    int force_fb_btn = 0;
    int last_left_btn = 0;
    int last_mid_btn = 0;
    int last_force_fb_btn = 0;
    int deadman = 0;
    int last_deadman = 0;
    
    int gripper_cmd = 0;
    
    // Set Haption start position values
    hap_start_pos_q[0] = 0.3; 	//x  --- in/out 	--- LIMITS:  0.2  / 0.4
    hap_start_pos_q[1] = 0.0; 	//y  --- left/right	--- LIMITS: -0.25 / 0.25	
    hap_start_pos_q[2] = 0.07;	//z  --- down/up	--- LIMITS: -0.16 / 0.31
    hap_start_pos_q[3] = 0.0;		//qx
    hap_start_pos_q[4] = 0.0;		//qy
    hap_start_pos_q[5] = 0.0;		//qz
    hap_start_pos_q[6] = 1.0;		//qw
    
    // Get Kuka start position in YPR and Quats
    std::vector<double> tmp;
    n.getParam("initial_kuka_pos", tmp);				// Get from kuka
    for (int i = 0; i < 6; ++i) {kuka_start_pos_ypr[i] = tmp[i];} 	// Convert to array
    YPR_2_Q(kuka_start_pos_ypr, kuka_start_pos_q);			// Convert to quaternions
 
    // Open connection to haption
    func_result = openConnectionToHaption(VC);
   
    // Make sure force feedback button is not pressed to start
    for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &buttons[i]);}
    force_fb_btn = buttons[2];
    bool force_fb_checker = false;
    std::cout<<"Force feedback button must be un-pressed to start"<<std::endl;
    while(!force_fb_checker)
    {
    	if(force_fb_btn==0) {force_fb_checker = true;}
    	else
    	{
    	    for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &buttons[i]);}
    	    force_fb_btn = buttons[2];
	}
    }
    std::cout<<"Starting control"<<std::endl;
    
    if (func_result == true)
    {
	    while(ros::ok())
	    {
	    	
	    	/*
	    	// Clutch control: If deadman pressed, update start positions
	    	virtGetDeadMan(VC, &deadman);
	    	if((deadman == 1) && (last_deadman == 0))
	    	{
	    		// Update haption start position
	    		virt_result = virtGetPosition(VC, hap_start_pos_q);
	    		
	    		// Update kuka start position
	    		n.getParam("kuka_pos", tmp);						// Get kuka pos
	    		for (int i = 0; i < 6; ++i) {kuka_start_pos_ypr[i] = tmp[i];}	// Build array
	    		YPR_2_Q(kuka_start_pos_ypr, kuka_start_pos_q);			// Convert to quaternions
	    	}	
	    	
	    	// Update deadman toggle
	    	last_deadman = deadman;	
	    	*/
	    		
	    	// Get Haption ee position [x, y, z, qx, qy, qz, qw]
	 	virt_result = virtGetPosition(VC, hap_pos_q);
						
		// Convert to Haption to Kuka workspace position (Quats) to generate move command
		// Translation: XYZ scaled movement [Move Start->P = (Base->P - Base->Start)]
		for(int i = 0; i < 3; ++i) {move_pos_q[i] = kuka_start_pos_q[i] + ws_factor * (hap_pos_q[i] - hap_start_pos_q[i]);}
		
	 	// Rotation: Match Haption Quaternion
		for(int i = 3; i < 7; ++i) {move_pos_q[i] = hap_pos_q[i];}  
		
	 	// Rotation: Clutch Control [ Rot(Start->P) = Rot(Start->Base) * Rot(Base->P) = inv(Rot(Base->Start))*Rot(Base->P) ]
	 	//for(int i = 3; i < 7; ++i) {move_pos_q[i] = kuka_start_pos_q[i] + hap_pos_q[i];}
	 	    	 
	    	// Publish move command to ros
		Publish_Move_Cmd_q(move_pos_q, move_pub_q);		
	 	    	
	    	// Get haption buttons input : 0/1 [left, mid, force_fb, right(?)]
	    	for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &buttons[i]);}
		
		left_btn = buttons[0];
		mid_btn = buttons[1];
		force_fb_btn = buttons[2];
		
	    	if ((mid_btn==1) && (last_mid_btn==0)) // if middle button pressed then toggle gripper control
	    	{
	    		// Toggle gripper control (0 = open, 1 = closed)
	    		if (gripper_cmd == 0) {gripper_cmd = 1;}
	    		else if (gripper_cmd == 1) {gripper_cmd = 0;}
	    		
	    		// Send command
	    		//n.setParam("gripper_cmd", gripper_cmd);
	    	}
	    	last_mid_btn = mid_btn;
		    		
		if (force_fb_btn==1)
		{	
		    	// Get delayed_ft_sensor reading & apply forces to haption
		    	n.getParam("ft_delay/fx",ft_delay[0]);
		    	n.getParam("ft_delay/fy",ft_delay[1]);
		    	n.getParam("ft_delay/fz",ft_delay[2]);	    	
		    	n.getParam("ft_delay/tx",ft_delay[3]);
		    	n.getParam("ft_delay/ty",ft_delay[4]);
		    	n.getParam("ft_delay/tz",ft_delay[5]);
						
			// Scale and Reverse force direction for x and z (copsim only)
			ft_delay[0] = -ft_delay[0]*ft_factor;
			ft_delay[1] =  ft_delay[1]*ft_factor;
			ft_delay[2] = -ft_delay[2]*ft_factor;
			ft_delay[3] = 0.0f;
			ft_delay[4] = 0.0f;
			ft_delay[5] = 0.0f;
			
			// Get forces within range
			// FX,FY,FZ must be in range -15.0-15.0f, TX,TY,TZ must be in range -1.0-1.0f	
			for(int i=0; i<3; i++)
			{
				if(ft_delay[i]>3.0f) {ft_delay[i]=3.0f;}
				if(ft_delay[i]<-3.0f) {ft_delay[i]=-3.0f;}
				if(ft_delay[i+3]>0.3f) {ft_delay[i+3]=0.3f;}
				if(ft_delay[i+3]<-0.3f){ft_delay[i+3]=-0.3f;}
			}
				
		    	// Apply forces to haption
			virt_result = virtSetForce(VC, ft_delay);		// Only works in COMMAND_TYPE_IMPEDANCE - doesnt work in COMMAND_TYPE_VIRTMECH
			
		    	// Get force input from the user and publish (virtGetForce only works in COMMAND_TYPE_VIRTMECH, not COMMAND_TYPE_IMPEDANCE
		    	//virt_result = virtGetForce(VC, ft_user); // Only works in COMMAND_TYPE_VIRTMECH - doesnt work in COMMAND_TYPE_IMPEDANCE
	    	}
	    	
	    	// Spin ROS
	    	ros::spinOnce();
		loop_rate.sleep();
	    	
	    }
	    
	    // Other virts to try
	    // virtGetSpeed(VC, speed)
	    // virtGetDeadMan(VC, &deadman)
	    // virtSetPosition(VC, pos);
	    // virtSetSpeed(VC, speed);
	    
	    
	    // Close connection to haption
	    func_result = closeConnectionToHaption(VC);
    } 
    
    return 0;
}
