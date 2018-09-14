cmake_minimum_required(VERSION 3.1)

find_package(PCL 1.7 REQUIRED)

include_directories(include include/ydlidar ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/ydlidar/CYdLidar.cpp src/ydlidar/serial.cpp src/ydlidar/ydlidar_driver.cpp src/ydlidar/unix_timer.cpp src/ydlidar/unix_serial.cpp src/pointcloud.cpp)

set(CMAKE_CXX_COMPILER /usr/bin/g++) # Use g++ compiler for C++
set(CMAKE_CXX_STANDARD 11) # Use C++11, for cmake >= 3.1
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(tankbot src/main.cpp)
target_sources(tankbot PRIVATE ${SOURCES})

target_link_libraries(tankbot rt pthread ${PCL_LIBRARIES})