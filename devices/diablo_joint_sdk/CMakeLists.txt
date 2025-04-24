cmake_minimum_required(VERSION 3.8)
project(diablo_joint_sdk)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
cmake_policy(SET CMP0074 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
# find_package(Threads REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# set(library_name ${PROJECT_NAME}_core)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    src/crc_verify.cpp
    src/serial_handle.cpp
    src/SerialPort/SerialPort.cpp
)

install(DIRECTORY include DESTINATION include)

ament_auto_package()