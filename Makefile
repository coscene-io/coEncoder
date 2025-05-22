SHELL := /bin/bash

ROS1_DISTRO := noetic
ROS2_DISTRO := foxy humble jazzy

ROS_WS := $(shell pwd)
export ROS_WS

ROS_BIN_PATH := /opt/ros/$(ROS_DISTRO)/bin

lint:
	/ros_entrypoint.sh $(ROS_BIN_PATH)/ament_cpplint --filter=-build/include_order include src
	/ros_entrypoint.sh $(ROS_BIN_PATH)/ament_uncrustify include src
	/ros_entrypoint.sh $(ROS_BIN_PATH)/ament_copyright include src
	/ros_entrypoint.sh $(ROS_BIN_PATH)/ament_cppcheck include src
	/ros_entrypoint.sh $(ROS_BIN_PATH)/ament_xmllint include src
	/ros_entrypoint.sh $(ROS_BIN_PATH)/ament_lint_cmake ./CMakeLists.txt

build:
ifeq ($(findstring $(ROS_DISTRO), $(ROS1_DISTRO)), $(ROS_DISTRO))
	/ros_entrypoint.sh catkin_make install
else ifeq ($(findstring $(ROS_DISTRO), $(ROS2_DISTRO)), $(ROS_DISTRO))
	/ros_entrypoint.sh colcon build --event-handlers console_direct+
else
	$(error Unsupported ROS_DISTRO: $(ROS_DISTRO))
endif
