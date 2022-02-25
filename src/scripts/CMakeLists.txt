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
   src/config/config_kuka_BL.py
   src/config/config_kuka_C.py
   src/config/config_kuka_TR.py   

   #src/input/joy_sub.py
   #src/input/falcon_sub.py
   #src/input/input_control.py
   #src/input/haption_delay_sub.py
   #src/input/haption_delay_sub_ypr.py
   
   src/input/haption/jpos_delay.py
      
   src/feedback/ft_delay.py   
   src/feedback/sim_ft_delay.py
   
   #src/feedback/vrep_cam1_sub.py
   #src/feedback/HUD.py
   #src/feedback/joy_vibrate.py

   #src/kuka/kuka_pos_delay.py

   #src/sim/sim_delayed_pos_cmd_pub.py
   #src/sim/sim_kuka_jpos_sub.py
   #src/sim/sim_kuka_pos_sub.py
   
   src/gui/kuka_gui.py
   src/gui/cam1.py
   src/gui/cam2.py
   
   src/user/blink_detector.py
   src/user/user_phys.py
   src/user/user_cam.py
   
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

add_executable(falcon_node src/input/falcon_pub.cpp)

add_dependencies(falcon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
   falcon_node
   nifalcon
   nifalcon_cli_base
   usb-1.0
   ${catkin_LIBRARIES}
)

