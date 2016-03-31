cmake_minimum_required(VERSION 2.8.3)
project(ros_gazebo_step_and_stop)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(ros_step_and_stop ros_step_and_stop.cpp)
target_link_libraries(ros_step_and_stop
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)
