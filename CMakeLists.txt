cmake_minimum_required(VERSION 3.0.2)
project(surface_reconstruction)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge tf )



catkin_package( CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf)

set(SR_SOURCE_FILES
        src/main.cpp
        src/mapping.cpp
        src/tracking.cpp
        src/vo_system.cpp
        )

set(SR_HEADER_FILSES
        include/surface_reconstruction/mapping.h
        include/surface_reconstruction/tracking.h
        include/surface_reconstruction/vo_system.h
        )

find_package(OpenCV REQUIRED)
include_directories(
        include ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        #${PCL_INCLUDE_DIRS}
        #  LIBRARIES imu_3dm_gx3
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

add_executable(surface_reconstruction ${SR_HEADER_FILES} ${SR_SOURCE_FILES})
target_link_libraries(surface_reconstruction ${OpenCV_LIBS} ${catkin_LIBRARIES})

