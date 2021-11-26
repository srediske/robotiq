cmake_minimum_required(VERSION 3.0.2)

project(robotiq_3f_gripper_visualization)

find_package(catkin REQUIRED urdf)

catkin_package()

install(
  DIRECTORY
    cfg
    meshes
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
