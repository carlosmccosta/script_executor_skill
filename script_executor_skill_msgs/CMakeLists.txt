cmake_minimum_required(VERSION 2.8.3)
project(script_executor_skill_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_action_files(
  FILES
  ScriptExecutorSkill.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
