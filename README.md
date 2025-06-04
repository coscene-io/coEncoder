# coEncoder

## Prerequisite

- Have ROS on your system

- Install dependencies
    ```bash
    sudo apt install libavformat-dev libswscale-dev libopencv-dev ros-{ros_distro}-foxglove-msgs -y
    ```

## Configuration

Edit the configuration parameters in the launch file (`coencoder/launch/coencoder.launch`):

- ROS1
    ```xml
    <launch>
        <!-- Specify topics requiring H264 encoding -->
        <rosparam param="/coencoder/subscribe_topics">['/camera_1', '/camera_2', '/camera_3']</rosparam>
        <rosparam param="/coencoder/video_resolutions">['1600x900','640x480','1280x720']</rosparam>
    
        <node name="coencoder" pkg="coencoder" type="coencoder" output="screen">
            <param name="output_fps" value="20"/>
            <param name="bitrate" value="400000"/>  
            <param name="depth_image_max_value" value="10000"/>
        </node>
    </launch>
    ```
- ROS2
    ```xml
    <launch>
        <node pkg="coencoder" exec="coencoder" name="coencoder" output="screen">
        <param name="output_fps" value="20"/>
        <param name="bitrate" value="400000"/>
        <param name="depth_image_max_value" value="10000"/>
        <param name="subscribe_topics" value="['/Node_1_image', '/Node_2_image']"/>
        <param name="video_resolutions" value="['1920*1080', '1920*1080']"/>
        </node>
    </launch>
    ```

- `subscribe_topics`: Specify one or more topics for H264 encoding. The topic's message types must be `sensor_msgs/CompressedImage` or `sensor_msgs/Image`.
- `video_resolutions`: Specify the resolution for each topic. Ensure a one-to-one correspondence with `subscribe_topics`.

## Compile

- ROS1
    ```bash
    # Copy the project into your ROS workspace
    cp -r {this_repo} {your_ros_ws}/src/
    
    # Source ROS setup and build
    source /opt/ros/{ros_distro}/setup.bash
  
    cd {your_ros_ws}
  
    catkin_make --pkg coencoder install
    ```
- ROS2
    ```bash
    # Copy the project into your ROS workspace
    cp -r {this_repo} {your_ros_ws}/src/
  
    # Source ROS setup and build
    source /opt/ros/{ros_distro}/setup.bash
  
    cd {your_ros_ws}
  
    colcon build --packages-select coencoder    
    ```

## RUN

- ROS1
    ```bash
    source {your_ros_ws}/install/setup.bash
    roslaunch coencoder coencoder.launch
    ```
  
- ROS2
    ```bash
    source {your_ros_ws}/install/setup.bash
    ros2 launch coencoder coencoder_launch.xml
    ```
