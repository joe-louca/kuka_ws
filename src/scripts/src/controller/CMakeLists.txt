cmake_minimum_required (VERSION 3.5.0)
project (tdpa)

include_directories(/opt/ros/noetic/include /opt/ros/noetic/lib)



#find_package(catkin REQUIRED COMPONENTS roscpp)
#catkin_package()

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                
	${PROJECT_SOURCE_DIR}/bin)


add_executable (tdpa TDPA.cpp)
add_executable (MyControl_V MyControl_V.cpp)
add_executable (MyControl_F MyControl_F.cpp)

