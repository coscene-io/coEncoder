cmake_minimum_required(VERSION 3.5)
project(coencoder)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Find FFmpeg packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)

find_package(OpenCV REQUIRED)

message(STATUS "ROS_VERSION: " $ENV{ROS_VERSION})
message(STATUS "ROS_DISTRO: " $ENV{ROS_DISTRO})
message(STATUS "ROS_ROOT: " $ENV{ROS_ROOT})

if("$ENV{ROS_VERSION}" STREQUAL "1")
    message(STATUS "Building with catkin")
    set(ROS_BUILD_TYPE "catkin")
    add_definitions(-DROS_VERSION_1)

    # Find catkin packages and dependencies
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_srvs
            sensor_msgs
            foxglove_msgs
    )

    catkin_package(
            CATKIN_DEPENDS roscpp std_srvs sensor_msgs foxglove_msgs
    )

    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )

    # Add the executable
    add_executable(coencoder src/ros1/coencoder.cpp)

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
            ${AVCODEC_LIBRARIES}
            ${AVFORMAT_LIBRARIES}
            ${AVUTIL_LIBRARIES}
            ${SWSCALE_LIBRARIES}
    )
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    message(STATUS "Building with ament_cmake")
    add_definitions(-DROS_VERSION_2)

    set(ROS_BUILD_TYPE "ament_cmake")

    find_package(ament_cmake REQUIRED)

    find_package(rclcpp REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(foxglove_msgs REQUIRED)

    add_executable(coencoder src/ros2/coencoder.cpp)

    ament_target_dependencies(coencoder rclcpp std_srvs sensor_msgs foxglove_msgs)


    target_include_directories(coencoder PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(coencoder
            ${OpenCV_LIBS}
            ${AVFORMAT_LIBRARIES}
            ${AVCODEC_LIBRARIES}
            ${AVUTIL_LIBRARIES}
            ${SWSCALE_LIBRARIES}
    )

else()
    message(FATAL_ERROR "ROS_VERSION environment variable must be 1 or 2")
endif()


if(ROS_BUILD_TYPE STREQUAL "catkin")
    install(TARGETS coencoder
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY src/ros1/launch
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            FILES_MATCHING PATTERN "*.launch"
    )
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
    install(TARGETS
            coencoder
            DESTINATION lib/${PROJECT_NAME})

    install(DIRECTORY src/ros2/launch
            DESTINATION share/${PROJECT_NAME}/
    )
    ament_package()
endif()
