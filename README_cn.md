# coEncoder

## 前置条件

- 系统上需要安装ROS

- 安装依赖
```bash
sudo apt install libavcodec-dev libavutil-dev libopencv-dev libcurl4 ros-{ros_distro}-foxglove-msgs -y
```
## GPU支持
coencoder目前支持使用GPU进行编码，目前支持以下编码器：
```C++
"h264_nvenc",    // NVIDIA NVENC
"h264_qsv",      // Intel Quick Sync
"h264_amf",      // AMD VCE
"h264_vaapi",    // VAAPI (Linux hardware acceleration)
```

## 配置

如果系统环境变量包含`HOME`，配置文件位于`$HOME/.config/coencoder/config.json`，否则配置文件位于`/tmp/coencoder/config/config.json`. 如果使用 `rosrun` (or `ros2 run`) 启动coEncoder, 可使用 --config-file 指定配置文件路径

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
      "output": "/camera_0/raw_image/h264",
      "encode_preset": "ultrafast",
      "encode_tune": "zerolatency"
    },
    {
      "bitrate": 1600000,
      "encoder_name": "libx264",
      "input": "/camera_1/raw_image",
      "output": "/camera_1/raw_image/h264",
      "encode_preset": "ultrafast",
      "encode_tune": "zerolatency"
    }
  ]
}
```
* **enable_by_default:** 是否默认启用编码
* **log_directory:** 日志文件路径
* **log_level:** 日志级别，可选值：Debug / Info / Warn / Error
* **topics_param:** 数组类型，包含3个字段
  * **bitrate:** 输出码率
  * **encoder_name:** 编码器名称，支持`h264_nvenc`、`h264_qsv`、`h264_amf`、`h264_vaapi`，同时也可以使用`libx264`通过CPU编码帧. 如果配置中缺失该字段,则使用 `libx264` 进行编码
  * **input:** 输入topic名称
  * **output:** 输出topic名称
  * **encode_preset:**
  
    | 有效值                    | 编码速度      | CPU使用率      | 质量/编码率           |
    |:---------------------------|:--------------|:---------------|:---------------------|
    | **ultrafast**  *(默认)*    | 🚀 极快        | 🟢 最低        | 🔴 最差（码率最高）    |
    | **superfast**              | 🚀 很快        | 🟢 很低        | 🔴 很差              |
    | **veryfast**               | ⚡ 快          | 🟢 低          | 🟠 略差              |
    | **faster**                 | 快            | 🟡 中等偏低     | 🟡 一般              |
    | **fast**                   | 中等偏快        | 🟡 中等        | 🟢 还可以             |
    | **medium**                 | 平衡          | 🟠 中等偏高     | 🟢 推荐默认           |
    | **slow**                   | 慢            | 🔴 高          | 🟢 好                |
    | **slower**                 | 更慢          | 🔴 高          | 🟢 更好              |
    | **veryslow**               | 极慢          | 🔴 最高        | 🟢 最佳压缩率         |
    | **placebo**                | 💀 极慢        | 🔴 极高        | 🟢 几乎无额外收益      |
  * **encode_tune:**
  
    | 调优选项                    | 作用                                                            | CPU使用率               |
    |:----------------------------|:----------------------------------------------------------------|:------------------------|
    | **film**                    | 针对高质量电影素材（保细节、抗噪）                                | 🔴 稍高                 |
    | **animation**               | 针对动画（锐化边缘）                                             | 🔴 稍高                 |
    | **grain**                   | 保留胶片颗粒（复杂度高）                                         | 🔴 显著升高             |
    | **stillimage**              | 针对静态图像                                                     | 🟡 一般                 |
    | **psnr / ssim**             | 用于质量测试（不推荐）                                            | 🔴 稍高                 |
    | **fastdecode**              | 便于快速解码（减少B帧等）                                        | 🟢 较低                 |
    | **zerolatency** *(默认)*    | 低延迟实时传输（去掉缓冲）                                        | 🟢 较低                 |

## 在线配置修改
**在线配置修改需要 coScout v1.1.8 或更高版本**
* 在线配置编辑
  * 组织设置 -> 设备 -> 设备配置  

    ![img_0](./img/device-config.png)
  * 编辑字段
  
    ![img_1](./img/config-setting.png)
  在设备配置中，如上图所示添加`coEncoder`字段。注意`coEncoder`是`plugin_config`的子字段。
  * 配置有效性
    * 配置必须包含`topics_param`字段，且该字段必须为数组类型。
    * `topics_param`中的元素必须包含三个字段：`input`、`output`、`bitrate`。`input`和`output`为字符串，`bitrate`为整数。
    * `encoder_name`、`encode_preset`和`encode_tune`字段为可选字段。如果在`topics_param`中未明确指定这些字段，coEncoder将使用h264编码的默认值

## 编译或deb安装

*** 您可以通过自行编译来安装CoEncoder。或者，我们也会提供.deb包进行安装。 ***
- 编译 
  - ROS1
    ```bash
    # 将项目复制到您的ROS工作空间
    cp -r {this_repo} {your_ros_ws}/src/
    
    # Source ROS设置并构建
    source /opt/ros/{ros_distro}/setup.bash
  
    cd {your_ros_ws}
  
    catkin_make --pkg coencoder install
    ```
  - ROS2
    ```bash
    # 将项目复制到您的ROS工作空间
    cp -r {this_repo} {your_ros_ws}/src/
    # Source ROS设置并构建
    source /opt/ros/{ros_distro}/setup.bash
  
    cd {your_ros_ws}
    colcon build --packages-select coencoder    
    ```
    
- deb安装
  ```bash
    dpkg -i ros-{ros distro}-coencoder_latest_{system arch}.deb
  ```

## 运行

- ROS1
  ```bash
  # 如果通过编译安装coencoder，source您的工作空间  
  source {your_ros_ws}/install/setup.bash
  # 如果通过deb安装coencoder，source ros
  source /opt/ros/{ros destro}/setup.bash
  
  roslaunch coencoder coencoder.launch
  # 也可使用 `rosrun` 启动节点
  rosrun coencoder coencoder --config-file {your_config_file_path}
  ```
  
- ROS2
  ```bash
  # 如果通过编译安装coencoder，source您的工作空间  
  source {your_ros_ws}/install/setup.bash
  # 如果通过deb安装coencoder，source ros
  source /opt/ros/{ros destro}/setup.bash
  
  ros2 launch coencoder coencoder_launch.xml
  # 也可使用 `rosrun` 启动节点  
  ros2 run coencoder coencoder -- --config-file {your_config_file_path}
  ``` 