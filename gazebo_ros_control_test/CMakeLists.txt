cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_control_test)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  controller_interface
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_ros_control
    controller_interface
  INCLUDE_DIRS
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Libraries
add_library(anti_gravity_controller src/anti_gravity_controller.cpp)
target_link_libraries(anti_gravity_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(wrench_robot_hw_sim src/wrench_robot_hw_sim.cpp)
target_link_libraries(wrench_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Install
install(TARGETS anti_gravity_controller wrench_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES robot_hw_sim_plugins.xml ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

