cmake_minimum_required(VERSION 2.8.3)
project(pose_stamp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_stamp
#  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(orb_posestamp src/orb_posestamp.cpp)
target_link_libraries(orb_posestamp ${catkin_LIBRARIES})
add_dependencies(orb_posestamp pose_stamp_generate_messages_cpp)

add_executable(pose2d_posestamp src/pose2d_posestamp.cpp)
target_link_libraries(pose2d_posestamp ${catkin_LIBRARIES})
add_dependencies(pose2d_posestamp pose_stamp_generate_messages_cpp)
