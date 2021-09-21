cmake_minimum_required (VERSION 3.5.0)
project (my_hap)

include_directories(/opt/ros/noetic/include /opt/ros/noetic/lib)



#find_package(catkin REQUIRED COMPONENTS roscpp)
#catkin_package()

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                
	${PROJECT_SOURCE_DIR}/bin)

link_directories(${PROJECT_SOURCE_DIR}/linux/lib) #virtuoseAPI.so
link_directories(${PROJECT_SOURCE_DIR}/linux/bin) #virtuoseAPI.a

add_executable (haption haption.cpp)
target_link_libraries(
        haption
        virtuose
        dl
        -I/opt/ros/noetic/include
        -L/opt/ros/noetic/lib
        -lroscpp
        -lrostime
        -lrosconsole
        -lroscpp_serialization
)
