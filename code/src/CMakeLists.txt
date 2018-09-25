# Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
# Hochschule Bonn-Rhein-Sieg University of Applied Sciences
# Version 1.0 
cmake_minimum_required(VERSION 2.8)
project(youbot)

find_package (Simbody REQUIRED)
find_package (Boost COMPONENTS filesystem date_time thread system REQUIRED)
find_package (orocos_kdl REQUIRED)

FILE(GLOB CommonCppSources safetycontroller.cpp pidcontroller.cpp movesynchronous.cpp)
FILE(GLOB CppSources main.cpp computedtorquecontrol.cpp kinematicchain.cpp dynamicmodel.cpp frictionobserver.cpp youbot.cpp)

FILE(GLOB SimbodySource main_simbody.cpp simbody.cpp kinematicchain.cpp dynamicmodel.cpp) #simbody.cpp
FILE(GLOB YoubotBase main_base.cpp youbot_base.cpp)

include_directories (${Simbody_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
include_directories (/usr/include/eigen3)
include_directories (
    /usr/local/include
    /usr/local/include/youbot_driver/soem/osal
	/usr/local/include/youbot_driver/soem/oshw/linux
)

add_executable (computedtorquecontrol_manipulator ${CommonCppSources} ${CppSources} ${PROJECT_INCLUDE_DIRS} )
add_executable (computedtorquecontrol_base ${CommonCppSources} ${YoubotBase} ${PROJECT_INCLUDE_DIRS} )
add_executable (computedtorquecontrol_simbody ${SimbodySource} ${CommonCppSources} ${PROJECT_INCLUDE_DIRS} )

target_link_libraries (computedtorquecontrol_manipulator YouBotDriver soem ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries (computedtorquecontrol_base YouBotDriver soem ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries (computedtorquecontrol_simbody ${Simbody_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
