cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)

project(spine)


add_library(dxl SHARED IMPORTED)
set_target_properties(dxl PROPERTIES
        IMPORTED_LOCATION /home/yuxuan/Downloads/encoder_motor_steering/DynamixelSDK/libdxl_x64_cpp.so
        INTERFACE_INCLUDE_DIRECTORIES "/home/yuxuan/Downloads/encoder_motor_steering/DynamixelSDK/include/dynamixel_sdk"
        )

add_library(${PROJECT_NAME} minicheetah_motor.cpp socketcan.cpp dynamixel_motor.cpp briter_encoder.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} pthread dxl)