#
# Author: Jakob Engel <jajuengel@gmail.com>
# Contributor: Stefan Wilkes <stefan.wilkes@gmail.com>
#
cmake_minimum_required(VERSION 2.8.3)
project(tum_ardrone)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ardrone_autonomy
    cv_bridge
    dynamic_reconfigure
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs
    message_generation
    roscpp
    rospy
)

# So that libcvd can compile properly; that's it.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

# Compile third party libs
include(ExternalProject)
ExternalProject_Add(thirdparty
    URL ${PROJECT_SOURCE_DIR}/thirdparty/thirdparty.tar.gz
    PREFIX ${CMAKE_BINARY_DIR}/thirdparty
    CONFIGURE_COMMAND ""
    BUILD_COMMAND make
    INSTALL_COMMAND ""
    BUILD_IN_SOURCE 1
)

# Add dynamic reconfigure API
generate_dynamic_reconfigure_options(
    cfg/AutopilotParams.cfg
    cfg/GUIParams.cfg
    cfg/StateestimationParams.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES filter_state.msg)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    SetReference.srv
    SetMaxControl.srv
    SetInitialReachDistance.srv
    SetStayWithinDistance.srv
    SetStayTime.srv
)

## Generate added messages
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS message_runtime std_msgs ardrone_autonomy)

###########
## Build ##
###########

add_subdirectory(src)
