cmake_minimum_required(VERSION 3.0.2)
project(calibration)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
# please install visp and visp ros as https://github.com/lagadic/visp_ros
find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(visp_ros REQUIRED)
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
)
add_executable(camera_calibration src/camera_calibration.cpp src/calibration-helper.hpp)

target_link_libraries(
  camera_calibration PUBLIC
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
)