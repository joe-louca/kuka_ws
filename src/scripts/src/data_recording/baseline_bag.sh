participant_id=$(rosparam get participant_id)
trial_id=$(rosparam get trial_id)  
latency=$(rosparam get latency)

mkdir -p ~/UserTrialData/P${participant_id}
#mkdir -p ~/RemoteHD/UserTrialData/P${participant_id}
rosbag record -O ~/UserTrialData/P${participant_id}/baseline.bag /user_phys
#mv -f ~/UserTrialData/P${participant_id}/baseline.bag ~/RemoteHD/UserTrialData/P${participant_id}/

