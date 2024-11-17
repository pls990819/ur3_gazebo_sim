cmake_minimum_required(VERSION 3.0.2)
project(ur3_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  rosconsole
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(PythonLibs 2.7)

add_message_files(
  FILES
  ParaTraj.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy roslib rosconsole std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS} 
  ${CROCODDYL_INCLUDE_DIRS}
)

add_executable(ur3_adaptive_controller
  src/adaptive_control_node.cpp
  src/ur3_controller.cpp
  src/traj_utils.cpp
  )
target_link_libraries(ur3_adaptive_controller ${PINOCCHIO_LIBRARIES} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(ur3_adaptive_controller PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(ur3_motion_planner
  src/motion_planner_node.cpp
  src/ur3_motion_planner.cpp
  src/traj_utils.cpp
  )
target_link_libraries(ur3_motion_planner ${PYTHON_LIBRARIES} ${CROCODDYL_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(ur3_motion_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
