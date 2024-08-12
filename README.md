# coEncoder

**Note:** This repository currently supports ROS1 only.

## Prerequisite

- Have ROS 1 on your system

```bash
sudo apt install libavcodec-dev
sudo apt install libopencv-dev
```

## Configuration

Edit the configuration parameters in the launch file (`coencoder/launch/coencoder.launch`):

```
<launch>
    <!-- Specify topics requiring H264 encoding -->
    <rosparam param="/coencoder/subscribe_topics">['/camera_1', '/camera_2', '/camera_3']</rosparam>
    <rosparam param="/coencoder/video_resolutions">['1600x900','640x480','1280x720']</rosparam>

    <node name="coencoder" pkg="coencoder" type="coencoder" output="screen">
        <param name="output_fps" value="20"/>
        <param name="bitrate" value="400000"/>
    </node>
</launch>
```

- `/coencoder/subscribe_topics`: Specify one or more topics for H264 encoding. The topics' message types must be `sensor_msgs/CompressedImage` or `sensor_msgs/Image`.
- `/coencoder/video_resolutions`: Specify the resolution for each topic. Ensure a one-to-one correspondence with `subscribe_topics`.

## Compile

```
# Copy the project into your ROS workspace
cp {this_repo} {your_ros_ws}/src/

# Source ROS setup and build
source /opt/ros/{ros_distro}/setup.bash
cd {your_ros_ws}
catkin_make install
```

## RUN

```
source {your_ros_ws}/install/setup.bash
roslaunch coencoder coencoder.launch
```
