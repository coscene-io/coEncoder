# coEncoder

## Prerequisite

- Have ROS on your system

- Install dependencies
```bash
sudo apt install libavcodec-dev libavutil-dev libopencv-dev libcurl4 ros-{ros_distro}-foxglove-msgs -y
```
## GPU Supported
coencoder currently supports encoding using GPUs, currently supports the following encoders:
```C++
"h264_nvenc",    // NVIDIA NVENC
"h264_qsv",      // Intel Quick Sync
"h264_amf",      // AMD VCE
"h264_vaapi",    // VAAPI (Linux hardware acceleration)
```

## Configuration

If the system environment variable contains `HOME`, the config file is located at `$HOME/.config/coencoder/config.json`, otherwise, the config file is located at `/tmp/coencoder/config/config.json`. If you start coEncoder using `rosrun` (or `ros2 run`), you can use --config-file to specify the config file path.

```Json
{
  "enable_by_default": true,
  "log_directory": "/home/cos/logs",
  "log_level": "Debug",
  "topics_param": [
    {
      "bitrate": 1600000,
      "encoder_name": "h264_nvenc",
      "input": "/camera_0/raw_image",
      "output": "/camera_0/raw_image/h264"
    },
    {
      "bitrate": 1600000,
      "encoder_name": "libx264",
      "input": "/camera_1/raw_image",
      "output": "/camera_1/raw_image/h264"
    }
  ]
}
```
* **enable_by_default**: Whether to enable encoding by default
* **log_directory**: Log file path
* **log_level**: Log level, possible values: Debug / Info / Warn / Error
* **topics_param**: Array type, contains 3 fields
  * **bitrate**: Output bitrate
  * **encoder_name**: Encoder name, supports `h264_nvenc`, `h264_qsv`, `h264_amf`, `h264_vaapi`, and you can also use `libx264` to encode frames by CPU. If this field is missing in the configuration, `libx264` will be used for encoding
  * **input**: Input topic name
  * **output**: Output topic name

## Online Configuration Modification
**Online configuration modification requires coScout v1.1.8 or later**
* Online configuration editing
  * Organization Settings -> Devices -> Device Configuration  

    ![img_0](./img/device-config.png)
  * Edit fields
  
    ![img_1](./img/config-setting.png)
  In device configuration, add the `coEncoder` field as shown in the image above. Note that `coEncoder` is a sub-field of `plugin_config`.
  * Configuration validity
    * The configuration MUST contain the `topics_param` field, and this field must be of array type.
    * Elements in `topics_param` MUST have three fields: `input`, `output`, `bitrate`. `input` and `output` are strings, `bitrate` is an integer.

## Compile OR deb install

*** You can install CoEncoder by compiling it yourself. Alternatively, we will also provide a .deb package for installation. ***
- Compile 
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
    
- deb Install
  ```bash
    dpkg -i ros-{ros distro}-coencoder_latest_{system arch}.deb
  ```

## RUN

- ROS1
  ```bash
  # if install coencoder by Compile, source your workspace  
  source {your_ros_ws}/install/setup.bash
  # if install coencoder by deb, source ros
  source /opt/ros/{ros destro}/setup.bash
  
  roslaunch coencoder coencoder.launch
  # You can also use `rosrun` to start the node
  rosrun coencoder coencoder --config-file {your_config_file_path}
  ```
  
- ROS2
  ```bash
  # if install coencoder by Compile, source your workspace  
  source {your_ros_ws}/install/setup.bash
  # if install coencoder by deb, source ros
  source /opt/ros/{ros destro}/setup.bash
  
  ros2 launch coencoder coencoder_launch.xml
  # You can also use `ros2 run` to start the node  
  ros2 run coencoder coencoder -- --config-file {your_config_file_path}
  ```
