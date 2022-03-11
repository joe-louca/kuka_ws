#include <iostream>
#include "haption/VirtuoseAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <sys/time.h>
#include <ctime>

#define _USE_MATH_DEFINES
 
#include <cmath>

#define HAPTION_CONTROLLER_IP "localhost" //"192.168.100.53"


VirtContext VC;

    bool openConnectionToHaption(VirtContext & VC)
    {
    	int r;
        VC = virtOpen("localhost");
        if (VC == NULL)
        {
            fprintf(stderr, "Error connecting to haption: %s\n", virtGetErrorMessage(virtGetErrorCode(NULL)));
            return false;
        }
        else
        {
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
            fprintf(stderr, "Error occurred in haption close: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));
            return false;
        }
        else {
            return true;
        }
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
    	
    	float QBqx = QB[3];
    	float QBqy = QB[4];
    	float QBqz = QB[5];
    	float QBqw = QB[6];
    	
    	// Leave X Y Z unchanged
    	QC[0] = QAx;	// X
    	QC[1] = QAy;	// Y
    	QC[2] = QAz;	// Z
    	
    	//Find rotation: QC = QA_inverse * QB
    	QC[3] = QAqw * QBqw - QAqx * QBqx - QAqy * QBqy - QAqz * QBqz; // qw
    	QC[4] = QAqw * QBqx + QAqx * QBqw + QAqy * QBqz - QAqz * QBqy; // qx
    	QC[5] = QAqw * QBqy - QAqx * QBqz + QAqy * QBqw + QAqz * QBqx; // qy
    	QC[6] = QAqw * QBqz + QAqx * QBqy - QAqy * QBqx + QAqz * QBqw; // qz     	
    }


    bool Quat2ZYX(float* q, float* ZYX)
    {
	float qx, qy, qz, qw, tmp1, tmp2, tmp3, tmp4, tmp5, rz, ry, rx;
	
	// Get individual quaternions
	qx = q[3];
	qy = q[4];
	qz = q[5];
	qw = q[6];	
	
   	// Ensure asin stays within -1 to 1 since asin of values larger than 1 produces complex number
	
	
	// Compute temp values
	tmp1 = 2*(qx*qy+qw*qz);
	tmp2 = qw*qw + qx*qx - qy*qy - qz*qz;
	//tmp3 = -2*(qx*qz-qw*qy);
	tmp4 = 2*(qy*qz+qw*qx);
	tmp5 = qw*qw - qx*qx - qy*qy + qz*qz;

   	// Ensure asin stays within -1 to 1 since asin of values larger than 1 produces complex number
    	tmp3 = -2*(qx*qz-qw*qy);
    	if (tmp3 >  1) {tmp3 = 1;}
    	if (tmp3 < -1) {tmp3 = -1;}
	
	rz = std::atan2(tmp1, tmp2);
	ry = std::asin (tmp3);
	rx = std::atan2(tmp4, tmp5);
	

	// Assemble ZYX
	ZYX[0] = q[0];
	ZYX[1] = q[1];
	ZYX[2] = q[2];
	ZYX[3] = rz;
	ZYX[4] = ry;
	ZYX[5] = rx;
	
	return true;
    }
    
    bool ZYX2Quat(float* ZYX, float* q)
    {
	float cz, cy, cx, sz, sy, sx, qx, qy, qz, qw, norm;
	
	// Get cos/sins of individual angles
	cz = cos(ZYX[3]/2);
	cy = cos(ZYX[4]/2);
	cx = cos(ZYX[5]/2);
	
	sz = sin(ZYX[3]/2);
	sy = sin(ZYX[4]/2);
	sx = sin(ZYX[5]/2);

	// Compute quaternion elements
	qx = cz*cy*sx-sz*sy*cx;
	qy = cz*sy*cx+sz*cy*sx;
	qz = sz*cy*cx-cz*sy*sx;
	qw = cz*cy*cx+sz*sy*sx;
	
	// Normalise quaternion
	norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
	qx /= norm;
	qy /= norm;
	qz /= norm;
	qw /= norm;	
	
	// Assemble quaternion
	q[0] = ZYX[0];
	q[1] = ZYX[1];
	q[2] = ZYX[2];
	q[3] = qx;
	q[4] = qy;
	q[5] = qz;
	q[6] = qw;
	
	return true;
    }
 
    float Q_Mult(float* a, float* b, float* c)
    {
    	float ax = a[0];
    	float ay = a[1];
    	float az = a[2];
    	float aw = a[3];
    	float bx = b[0];
    	float by = b[1];
    	float bz = b[2];
    	float bw = b[3];

	c[0] = aw * bx + ax * bw + ay * bz - az * by;
	c[1] = aw * by - ax * bz + ay * bw + az * bx;
	c[2] = aw * bz + ax * by - ay * bx + az * bw;
	c[3] = aw * bw - ax * bx - ay * by - az * bz;

	return true;
    }
    
    
    void Publish_Move_Cmd_q(float* cmd, ros::Publisher move_pub_q, float gripper_data, float clutch_counter, float frame_id, float timestamp)
    {  // Publish cmd as Float32MultiArray - followed by gripper_cmd, clutch_counter, frame_id, timestamp
  	std_msgs::Float32MultiArray cmd_msg;
  	cmd_msg.data.clear(); 
    	for (int i = 0; i < 7; ++i) {cmd_msg.data.push_back(cmd[i]);}   
    	
    	cmd_msg.data.push_back(gripper_data);
    	cmd_msg.data.push_back(clutch_counter);
    	cmd_msg.data.push_back(frame_id);
    	cmd_msg.data.push_back(timestamp);
    	
      	move_pub_q.publish(cmd_msg);
    }

    
int main(int argc, char* argv[])
{

    ros::init(argc, argv, "haption_stream");
    ros::NodeHandle n;
    ros::Publisher move_pub_q = n.advertise<std_msgs::Float32MultiArray>("hap_move_pub_q", 1);
      
    int rate_hz;
    n.getParam("rate_hz",rate_hz);
    //rate_hz = rate_hz*1.5;
    ros::Rate loop_rate(rate_hz);
    
    // Set some scale factors
    double ws_factor = 1.0; // ratio of haption:kuka move
    double ft_factor = 1.0; // scaling factor for forces to apply to haption
    double ft_user_scale = 0.5; 	// ft scale factor from gui
    double ws_user_scale = 0.5; 	// ft scale factor from gui
    
    // Declare some frame markers for data recording.
    float frame_id = 0;
    //double start_time = ros::Time::now().toSec();
    //n.setParam("start_time", start_time);
    double start_time;
    n.getParam("start_time", start_time); // float in secs
    double timestamp = ros::Time::now().toSec() - start_time;
    float clutch_counter = 0;
    float gripper_data = 0;
    
    bool func_result;
    int virt_result;

    // Declare some position arrays
    float hap_pos_q[7] = {};
    float hap_pos_ypr[6] = {};
    float hap_start_pos_q[7] = {};   
    float hap_start_pos_ypr[6] = {};   
    float hap_move_q[7] = {};
    float hap_move_ypr[6] = {};
    
    float kuka_start_pos_q[7] = {};

    float move_pos_ypr[6] = {};
    float move_pos_q[7] = {};

    float hap_force_ypr[6] = {};
    float ft_delay[6] = {};
    float ft_input[6] = {};
    float ft_user[6] = {};
    
    float Q_delta[4] = {};
    float Q_hap_current[4] = {};
    float Q_hap_start[4] = {};
    float Q_kuka_start[4] = {};
    float Q_move[4] = {};

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
    n.setParam("gripper_cmd", gripper_cmd);
    
    // Set Haption start position values
    hap_start_pos_q[0] = 0.3; 	//x  --- in/out 	--- LIMITS:  0.2  / 0.4
    hap_start_pos_q[1] = 0.0; 	//y  --- left/right	--- LIMITS: -0.25 / 0.25	
    hap_start_pos_q[2] = 0.07;	//z  --- down/up	--- LIMITS: -0.16 / 0.31
    hap_start_pos_q[3] = 0.0;		//qx
    hap_start_pos_q[4] = 0.0;		//qy
    hap_start_pos_q[5] = 0.0;		//qz
    hap_start_pos_q[6] = 1.0;		//qw
    
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
    
    // Click deadman to send start position to move slowly to

    while (true)
    {
    	virtGetDeadMan(VC, &deadman);
    	if(deadman == 1)
    	{
    		//Send one kuka pos then wait
    		virt_result = virtGetPosition(VC, hap_start_pos_q);
    		
    		// Update kuka start position from copelliasim
	    	n.getParam("sim_kuka_pos/x", kuka_start_pos_q[0]);
	    	n.getParam("sim_kuka_pos/y", kuka_start_pos_q[1]);
	    	n.getParam("sim_kuka_pos/z", kuka_start_pos_q[2]);
	    	n.getParam("sim_kuka_pos/qx", kuka_start_pos_q[3]);
	    	n.getParam("sim_kuka_pos/qy", kuka_start_pos_q[4]);
	    	n.getParam("sim_kuka_pos/qz", kuka_start_pos_q[5]);
	    	n.getParam("sim_kuka_pos/qw", kuka_start_pos_q[6]);
	    	
	    	// Make first move commandRotation: No translation, Match Haption Quaternion
	    	for(int i = 0; i < 3; ++i) {move_pos_q[i] = kuka_start_pos_q[i];}
		for(int i = 3; i < 7; ++i) {move_pos_q[i] = hap_start_pos_q[i];} 

    		break;
    	}
    }
    std::cout<<"Release the deadman switch"<<std::endl;
    while(true)
    {
        virtGetDeadMan(VC, &deadman);
    	if(deadman == 0) {break;}
    }
    
    
    Publish_Move_Cmd_q(move_pos_q, move_pub_q, gripper_data, clutch_counter, frame_id, timestamp);
    sleep(0.5); // wait 0.5 secs
    std::cout<<"Press the deadman switch to move continously"<<std::endl;
    
    if (func_result == true)
    {
	    while(ros::ok())
	    {
	    	// FOR TESTING
	    	// Update kuka start position from copelliasim
	    	n.getParam("sim_kuka_pos/x", move_pos_q[0]);
	    	n.getParam("sim_kuka_pos/y", move_pos_q[1]);
	    	n.getParam("sim_kuka_pos/z", move_pos_q[2]);
	    	n.getParam("sim_kuka_pos/qx", move_pos_q[3]);
	    	n.getParam("sim_kuka_pos/qy", move_pos_q[4]);
	    	n.getParam("sim_kuka_pos/qz", move_pos_q[5]);
	    	n.getParam("sim_kuka_pos/qw", move_pos_q[6]);
	    	move_pos_q[2] -= 0.01;
	    	Publish_Move_Cmd_q(move_pos_q, move_pub_q, gripper_data, clutch_counter, frame_id, timestamp);
	    	
	    	//
	    	
	    	
	    	// Clutch control: If deadman pressed, update start positions
	    	virtGetDeadMan(VC, &deadman);
	    	if((deadman == 1) && (last_deadman == 0))
	    	{
	    		clutch_counter += 1;
	    		
	    		// Update haption start position
	    		virt_result = virtGetPosition(VC, hap_start_pos_q);
	    		
	    		// Update kuka start position from copelliasim
	    		n.getParam("sim_kuka_pos/x", kuka_start_pos_q[0]);
	    		n.getParam("sim_kuka_pos/y", kuka_start_pos_q[1]);
	    		n.getParam("sim_kuka_pos/z", kuka_start_pos_q[2]);
	    		n.getParam("sim_kuka_pos/qx", kuka_start_pos_q[3]);
	    		n.getParam("sim_kuka_pos/qy", kuka_start_pos_q[4]);
	    		n.getParam("sim_kuka_pos/qz", kuka_start_pos_q[5]);
	    		n.getParam("sim_kuka_pos/qw", kuka_start_pos_q[6]);
	    	}	
	    	
	    	// Update deadman toggle
	    	last_deadman = deadman;	

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
	    		n.setParam("gripper_cmd", gripper_cmd);
	    	}
	    	last_mid_btn = mid_btn;

	    	
	    	// Only send move commands if deadman is pressed
	    	if(deadman ==1)
	    	{
		    	// Get Haption ee position [x, y, z, qx, qy, qz, qw]
		 	virt_result = virtGetPosition(VC, hap_pos_q);
							
			// Convert to Haption to Kuka workspace position (Quats) to generate move command
			// Translation: XYZ scaled movement [Move Start->P = (Base->P - Base->Start)]
			n.getParam("ws_user_scale",ws_user_scale);
			for(int i = 0; i < 3; ++i) {move_pos_q[i] = kuka_start_pos_q[i] + ws_factor * 1000 * (hap_pos_q[i] - hap_start_pos_q[i])*(ws_user_scale/100.0);} // mm
			
		 	// Rotation: Match Haption Quaternion
			for(int i = 3; i < 7; ++i) {move_pos_q[i] = hap_pos_q[i];}  
			
		 	// Rotation: Clutch Control - need to add conversion to forces if you use this
		 	// Build quaternion vectors
		 	//for(int i = 3; i < 7; ++i) {Q_hap_start[i-3] = hap_start_pos_q[i];}  
		 	//for(int i = 3; i < 7; ++i) {Q_hap_current[i-3] = hap_pos_q[i];}
		 	//for(int i = 3; i < 7; ++i) {Q_kuka_start[i-3] = kuka_start_pos_q[i];}  
		 	
		 	// Invert Q_hap_start
		 	//for(int i = 0; i < 3; ++i) {Q_hap_start[i] = -Q_hap_start[i];}  
		 	
		 	// Calculate differences		 	
			//Q_Mult(Q_hap_current, Q_hap_start, Q_delta);
			//Q_Mult(Q_delta, Q_kuka_start, Q_move);
			
			// Insert into move vector
			//for(int i = 3; i < 7; ++i) {move_pos_q[i] = Q_move[i-3];}  
		    			 	
		 	// Update data markers
		 	gripper_data = static_cast<float>(gripper_cmd);
		 	frame_id += 1;
    			timestamp = ros::Time::now().toSec() - start_time;

		    	// Publish move command to ros
			Publish_Move_Cmd_q(move_pos_q, move_pub_q, gripper_data, clutch_counter, frame_id, timestamp);		
		 	    	
		    	// Get haption buttons input : 0/1 [left, mid, force_fb, right(?)]
    			for (int i = 0; i < 5; i++) {virtGetButton(VC, i+1, &buttons[i]);}
			force_fb_btn = buttons[2];
			if (force_fb_btn==1)
			{		    	
			    	// Get delayed_ft_sensor reading & apply forces to haption
			    	n.getParam("ft_delay/fx",ft_delay[0]);
			    	n.getParam("ft_delay/fy",ft_delay[1]);
			    	n.getParam("ft_delay/fz",ft_delay[2]);	    	
			    	n.getParam("ft_delay/tx",ft_delay[3]);
			    	n.getParam("ft_delay/ty",ft_delay[4]);
			    	n.getParam("ft_delay/tz",ft_delay[5]);
						
				//ft_delay[0] = 0.1;
				//ft_delay[1] = 0.1;
				//ft_delay[2] = 0.1;
				//ft_delay[3] = 0.1;
				//ft_delay[4] = 0.1;
				//ft_delay[5] = 0.1;
								
				// Scale and Reverse force direction for x and z (copsim only)
				/*
				ft_delay[0] = -ft_delay[0]*ft_factor;
				ft_delay[1] =  ft_delay[1]*ft_factor;
				ft_delay[2] = -ft_delay[2]*ft_factor;
				ft_delay[3] = 0.0f;
				ft_delay[4] = 0.0f;
				ft_delay[5] = 0.0f;
				*/
				
				// Scale force direction
				n.getParam("ft_user_scale",ft_user_scale);
				//ft_user_scale = 100;
				ft_delay[0] = ft_delay[0]*ft_factor*(ft_user_scale/100.0);
				ft_delay[1] = ft_delay[1]*ft_factor*(ft_user_scale/100.0);
				ft_delay[2] = ft_delay[2]*ft_factor*(ft_user_scale/100.0);
				ft_delay[3] = ft_delay[3]*ft_factor;//*0.5*(ft_user_scale/100.0);
				ft_delay[4] = ft_delay[4]*ft_factor;//*0.5*(ft_user_scale/100.0);
				ft_delay[5] = ft_delay[5]*ft_factor;//*0.5*(ft_user_scale/100.0);
				
				// If using orientation clutch control - convert FTs relative to hap start q
				// W_R_start = Q2Rot(hap_start_q)
				// W_R_start * ft_delay(0-2)
				// W_R_start * ft_delay(3-5)
				
				// Get forces within safe range
				for(int i=0; i<3; i++)
				{
					if(ft_delay[i]>5.0f) {ft_delay[i]=5.0f;}
					if(ft_delay[i]<-5.0f) {ft_delay[i]=-5.0f;}
					if(ft_delay[i+3]>0.3f) {ft_delay[i+3]=0.3f;}
					if(ft_delay[i+3]<-0.3f){ft_delay[i+3]=-0.3f;}
				}
					
			    	// Apply forces to haption
				//virt_result = virtSetForce(VC, ft_delay);		// Only works in COMMAND_TYPE_IMPEDANCE - doesnt work in COMMAND_TYPE_VIRTMECH
				virt_result = virtAddForce(VC, ft_delay);
				
			    	// Get force input from the user and publish (virtGetForce only works in COMMAND_TYPE_VIRTMECH, not COMMAND_TYPE_IMPEDANCE
		    		//virt_result = virtGetForce(VC, ft_user); // Only works in COMMAND_TYPE_VIRTMECH - doesnt work in COMMAND_TYPE_IMPEDANCE
		    		
		    		last_force_fb_btn = 1;
		    	}
	    	}
	    	// Reset connection if force_fb or deadman unpressed
	    	if (((last_force_fb_btn==1)&&(force_fb_btn==0)))// || ((last_deadman==1)&&(deadman==0)))
		{
			std::cout<<"Turning off force feedback..."<<std::endl;
			func_result = closeConnectionToHaption(VC);
			sleep(0.01);
			func_result = openConnectionToHaption(VC);
			sleep(0.01);
			std::cout<<"Force feedback off"<<std::endl;
			
		}	
		last_force_fb_btn = force_fb_btn;
		//last_deadman = deadman;
	    	
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
