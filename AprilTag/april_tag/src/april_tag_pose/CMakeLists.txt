cmake_minimum_required(VERSION 3.4)
project(april_tag_pose)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(visp_ros REQUIRED)
message("cmake_module_path: " ${CMAKE_MODULE_PATH})
# find_package(VISP_ROS REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES april_tag_pose
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${visp_ros_INCLUDE_DIRS}
)
add_executable(test_camera src/test_camera.cpp)

target_link_libraries(
  test_camera PRIVATE 
  depthai::opencv 
  ${OpenCV_LIBS}
  ${VISP_LIBRARIES}
)

add_executable(test_pose src/test_pose.cpp)

target_link_libraries(
  test_pose PRIVATE
  depthai::opencv
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${visp_ros_LIBRARIES}
)