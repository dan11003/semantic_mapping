cmake_minimum_required(VERSION 2.8.3)
project(semantic_map)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-std=c++11)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    geometry_msgs
    cmake_modules
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf_conversions
    tf
    genmsg
    tf_conversions
    )
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED COMPONENTS)


catkin_package(
    DEPENDS Eigen libpcl-all-dev
    CATKIN_DEPENDS  roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs cmake_modules eigen_conversions pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
src/semanticscreator.cpp
)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_executable(semantic_map_node src/semantic_map_fake_publisher.cpp )
target_link_libraries(semantic_map_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

#  ${Eigen_LIBRARIES}
#)

