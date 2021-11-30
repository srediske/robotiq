cmake_minimum_required(VERSION 3.0.2)

project(robotiq_2f_85_gripper_articulated_gazebo)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
