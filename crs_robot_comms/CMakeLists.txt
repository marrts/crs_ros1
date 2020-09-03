cmake_minimum_required(VERSION 2.8.3)
project(crs_robot_comms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  controller_manager_msgs
  ur_dashboard_msgs
  ur_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
    std_srvs
    controller_manager_msgs
    ur_dashboard_msgs
    ur_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME}_robot_comms_server src/robot_comms_server.cpp)

target_include_directories(${PROJECT_NAME}_robot_comms_server PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  )
target_link_libraries(
  ${PROJECT_NAME}_robot_comms_server
  ${catkin_LIBRARIES}
)
