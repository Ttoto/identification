cmake_minimum_required(VERSION 2.8.3)
project(identification)



## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    )
find_package (OpenCV 3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )


add_executable(map_recorder src/map_recorder.cpp)
add_executable(identify src/identify.cpp)

target_link_libraries(identify
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

target_link_libraries(map_recorder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
