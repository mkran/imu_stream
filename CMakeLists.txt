# Set required cmake version
cmake_minimum_required(VERSION 2.8.3)

# Project name
project(imu_stream)

# Create variable containing all source files
set(SOURCE_FILES
src/attitude.cpp
src/error_detection.cpp
src/packet.cpp
src/serialport.cpp
src/event.cpp
src/packetfinder.cpp
src/thread.cpp
src/compositedata.cpp
src/ezasyncdata.cpp
src/port.cpp
src/types.cpp
src/conversions.cpp
src/position.cpp
src/util.cpp
src/criticalsection.cpp
src/searcher.cpp
src/utilities.cpp
src/dllvalidator.cpp
src/memoryport.cpp
src/sensors.cpp
src/vntime.cpp
)

## Find catkin macros and libraries used
find_package(catkin REQUIRED roscpp geometry_msgs tf)

# Include header files in the "include" directory
include_directories(include ${catkin_INCLUDE_DIRS})

# Specify catkin-specific information to the build system to generate pkg-config and CMake files
catkin_package()

# Create executable for the node from the source files and main function
add_executable(
imu_stream
src/imu_stream.cpp
${SOURCE_FILES}
)

# Link catkin libraries
target_link_libraries(imu_stream ${catkin_LIBRARIES})

