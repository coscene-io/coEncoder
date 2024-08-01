# CoEncoder
* caution: this repo only support ROS1 now
## install depends
```
sudo apt install libavcodec-dev
sudo apt install libopencv-dev
```
## Config
Config parameters in launch file (coencoder/launch/coencoder.launch)
```
<launch>
    <node name="coencoder" pkg="coencoder" type="coencoder" output="screen">
      # Specify one or more topics that require h264 encoding
      <rosparam param="topics">['/camera_1', '/camera_2', '/camera_3']</rosparam>
      <rosparam param="resolutions">['1600*900','640*480','1280*720']</rosparam>
      <param name="output_fps"          value="20"/>
      <param name="bitrate"             value="400000"/>
    </node>
</launch>
```
In param `topics`, you can specify one or more topics that require h264 encoding, certenly, those topic's message type must be sensor_msgs/CompressedImage or sensor_msgs/Image.
In param `resolutions`, you need input those image's size, because encoder's context need it.

for sure, `topics` and `resolutions` has to be one-to-one.

## Compile
```
# copy this project into {your_ros_ws}/src/
cp {this_repo} {your_ros_ws}/src/

# source and catkin_make
source /opt/ros/{ros_distro}/setup.bash
cd {your_ros_ws}
catkin_make install
```

## RUN
```
source {your_ros_ws}/install/setup.bash
roslaunch coencoder coencoder.launch
```
