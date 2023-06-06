cmake_minimum_required(VERSION 3.4)
project(april_tag_pose)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
# please install visp and visp ros as https://github.com/lagadic/visp_ros
find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(visp_ros REQUIRED)
find_package(Eigen3 REQUIRED)
message("cmake_module_path: " ${CMAKE_MODULE_PATH})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES april_tag_pose
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${visp_ros_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_estimation src/pose_estimation.cpp)

target_link_libraries(
  pose_estimation PUBLIC
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${visp_ros_LIBRARIES}
  )

add_executable(open_cam src/open_cam.cpp)

target_link_libraries(
  open_cam PUBLIC
  depthai::opencv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

catkin_install_python(PROGRAMS
  scripts/pose_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
