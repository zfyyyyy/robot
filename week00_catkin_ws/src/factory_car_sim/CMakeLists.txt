cmake_minimum_required(VERSION 3.0.2)
project(factory_car_sim)
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  tf2_ros
  urdf
  xacro
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(move_car_node src/move_car.cpp)
target_link_libraries(move_car_node
  ${catkin_LIBRARIES}
)
catkin_install_python(PROGRAMS
  scripts/cmd_vel_follower.py
  scripts/wasd_teleop.py
  scripts/tf_chain_follower.py
  scripts/model_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)