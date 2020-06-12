cmake_minimum_required(VERSION 3.5)
project(crs_support_ros1)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    find_package(catkin REQUIRED)
    catkin_package()
    set(ROS_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    find_package(ament_cmake REQUIRED)
    ament_package()
    set(ROS_SHARE_DESTINATION share)
endif()


install(DIRECTORY
    launch
    DESTINATION ${ROS_SHARE_DESTINATION}/${PROJECT_NAME}/
)
