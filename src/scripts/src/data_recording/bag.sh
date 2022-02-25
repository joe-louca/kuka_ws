participant_id=$(rosparam get participant_id)
trial_id=$(rosparam get trial_id)  
latency=$(rosparam get latency)

mkdir /media/joe/My\ Passport/data/P${participant_id}

rosbag record -O /media/joe/My\ Passport/data/P${participant_id}/P${participant_id}_H${trial_id}_${latency}ms.bag  /hap_move_pub_q #/ft_delay #/user_phys #/cameras/cam1 /user_phys #/cameras/user_cam 


## Converting images to video
#roslaunch launch export.launch 								# extracts jpegs of each frame
#mv ~/.ros/frame*.jpg /media/joe/My\ Passport/data/extracted/ 				# move frames to a new folder for handling
#rm ~/.ros/frame*.jpg 										# delete all frames in .ros
#cd /media/joe/My\ Passport/data/extracted							# go to directory with the extracted images
#ffmpeg -framerate 10 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p /media/joe/My\ Passport/data/trial_ID.mp4 # convert to mp4
#rm /media/joe/My\ Passport/data/extracted/frame*.jpg 						# delete all frames in extracted directory

