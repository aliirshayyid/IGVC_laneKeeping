cmake_minimum_required(VERSION 2.8.3)
project(line_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)
find_package(OpenCV 3.2.0 REQUIRED)

# Running the cfg files here 
generate_dynamic_reconfigure_options(
  cfg/Lane_following.cfg
)

# generate_dynamic_reconfigure_options(
#   cfg/test_q2.cfg
# )

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_following
  src/line_following_node.cpp
  src/line_following.cpp
)
# adding the dependencies that are required to run the cfg file here 
  add_dependencies(line_following
  ${PROJECT_NAME}_gencfg
  )

# add_dependencies(line_following
#   ${PROJECT_NAME}_gencfg
# )
target_link_libraries(line_following
  ${catkin_LIBRARIES}
)

