cmake_minimum_required(VERSION 3.0.2)
project(script_executor_skill_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  script_executor_skill_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy actionlib script_executor_skill_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS nodes/script_executor_skill.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
