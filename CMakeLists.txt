cmake_minimum_required(VERSION 3.10)

# Project name
project(VideoFilter)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GSTREAMER_INCLUDE_DIRS})
include_directories(${GST_APP_INCLUDE_DIRS})

# Link directories
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})
link_directories(${GST_APP_LIBRARY_DIRS})

# Add executable
add_executable(VideoFilter main.cpp)

# Link libraries
target_link_libraries(VideoFilter ${OpenCV_LIBRARIES})
target_link_libraries(VideoFilter ${GSTREAMER_LIBRARIES})
target_link_libraries(VideoFilter ${GST_APP_LIBRARIES})
