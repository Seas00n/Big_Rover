cmake_minimum_required(VERSION 3.4)
project(april_tag_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES april_tag_pose
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(test_camera src/test_camera.cpp)

target_link_libraries(
  test_camera PRIVATE 
  depthai::opencv 
  ${OpenCV_LIBS}
  ${VISP_LIBS}
)