cmake_minimum_required(VERSION 3.0.2)
project(crane_anticollision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  crane_msg
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES crane_anticollision_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf2_ros crane_msg
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(crane_anticollision_core
  src/anti_collision_core.cpp
)
target_link_libraries(crane_anticollision_core Eigen3::Eigen)

add_executable(multi_crane_anticollision_node src/multi_crane_anticollision_node.cpp)
target_link_libraries(multi_crane_anticollision_node crane_anticollision_core ${catkin_LIBRARIES})
add_dependencies(multi_crane_anticollision_node crane_msg_generate_messages_cpp)

add_executable(multi_crane_anticollision_viz_node src/multi_crane_anticollision_viz_node.cpp)
target_link_libraries(multi_crane_anticollision_viz_node ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(multi_crane_anticollision_viz_node crane_msg_generate_messages_cpp)

add_executable(crane_teleop_evdev_node src/crane_teleop_evdev_node.cpp)
target_link_libraries(crane_teleop_evdev_node ${catkin_LIBRARIES})
add_dependencies(crane_teleop_evdev_node crane_msg_generate_messages_cpp)
