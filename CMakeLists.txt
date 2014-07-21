cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_saver)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_saver_node src/pointcloud_saver_node.cpp)
target_link_libraries(pointcloud_saver_node ${catkin_LIBRARIES})

install(TARGETS pointcloud_saver_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
