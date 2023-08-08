participant_id=$(rosparam get participant_id)
trial_id=$(rosparam get trial_id)  
latency=$(rosparam get latency)

mkdir -p ~/UserTrialData/P${participant_id}
#mkdir -p ~/RemoteHD/UserTrialData/P${participant_id}
rosbag record -O ~/UserTrialData/P${participant_id}/T${trial_id}.bag /v_hap_out /F_kuka_out /kuka_pos
#mv -f ~/UserTrialData/P${participant_id}/T${trial_id}.bag ~/RemoteHD/UserTrialData/P${participant_id}/

# records haption command, kuka forces, actual path of the arm, user physiology (for trust & workload)
