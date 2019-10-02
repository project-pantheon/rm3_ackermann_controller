cmake_minimum_required(VERSION 2.8.3)
project(rm3_ackermann_controller)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  cmake_modules
)

find_package(Eigen REQUIRED)

add_service_files(
   FILES
   SetKvalues.srv
   ActivateController.srv
   GetTree.srv
)

generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy message_runtime std_srvs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(ackermann_controller src/ackermann_controller.cpp)

add_dependencies(ackermann_controller
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ackermann_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)


add_executable(tree_manager src/tree_manager.cpp)

target_link_libraries(tree_manager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)





