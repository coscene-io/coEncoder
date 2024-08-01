cmake_minimum_required(VERSION 3.0.2)
project(coencoder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin packages and dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        foxglove_msgs
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find FFmpeg packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale)

if (FFMPEG_FOUND)
    include_directories(${FFMPEG_INCLUDE_DIRS})
    link_directories(${FFMPEG_LIBRARY_DIRS})
    message(STATUS "FFmpeg includes: ${FFMPEG_INCLUDE_DIRS}")
    message(STATUS "FFmpeg libraries: ${FFMPEG_LIBRARIES}")
else()
    message(FATAL_ERROR "FFmpeg not found")
endif()

# Declare catkin package
catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs foxglove_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${FFMPEG_INCLUDE_DIRS}
)

# Add the executable
add_executable(coencoder src/coencoder.cpp)

target_include_directories(coencoder
        SYSTEM PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${catkin_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(coencoder
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${FFMPEG_LIBRARIES}
)

install(TARGETS coencoder
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)

