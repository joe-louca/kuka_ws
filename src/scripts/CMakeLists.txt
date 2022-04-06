cmake_minimum_required(VERSION 3.0.2)
project(scripts)

set(CMAKE_CXX_FLAGS "-std=c++14 -lnifalcon")#Falcon lib is build agains c++14


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  moveit_commander
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

include_directories(/opt/ros/noetic/include /opt/ros/noetic/lib)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                
	${PROJECT_SOURCE_DIR}/bin)
	
link_directories(${PROJECT_SOURCE_DIR}/linux/lib) #virtuoseAPI.so
link_directories(${PROJECT_SOURCE_DIR}/linux/bin) #virtuoseAPI.a

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  moveit_commander
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  iiwaPy
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
   src/config/config_kuka.py
   src/config/timestep.py
      
   # Inputs
   src/input/joy/joy_sub.py
   # Falcon (C++)
   # Haption (C++)
   
   # Topic Delays
   src/delay/F_kuka_out_delay.py
   src/delay/delayed_F_kuka_out_delay.py
   src/delay/v_hap_out_delay.py
   src/delay/delayed_v_hap_out_delay.py
   src/delay/gripper_cmd_delay.py
   src/delay/timestep_delay.py
   
   # Feedback
   src/feedback/ft_compensation.py
   src/feedback/sim_ft.py
   src/feedback/joy_vibrate.py

   # GUI & Cameras
   src/gui/kuka_gui.py
   src/gui/cam1.py
   src/gui/cam2.py
   src/gui/cam3.py
   
   # User Monitoring
   src/user/blink_detector.py
   src/user/user_phys.py
   src/user/user_cam.py
   
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Add falcon executable
add_executable(falcon_node src/input/falcon/falcon.cpp)
add_dependencies(falcon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(falcon_node 
  nifalcon 
  nifalcon_cli_base 
  usb-1.0 
  ${catkin_LIBRARIES}
)

# Add TDPA executable
add_executable(TDPA_node src/controller/TDPA.cpp)
add_dependencies(TDPA_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TDPA_node 
  ${catkin_LIBRARIES}
)
# Add NoControl executable
add_executable(NoControl_node src/controller/NoControl.cpp)
add_dependencies(NoControl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(NoControl_node 
  ${catkin_LIBRARIES}
)



## Specify libraries to link a library or executable target against


