# coEncoder

## 前置条件

- 系统上需要安装ROS

- 安装依赖
```bash
sudo apt install libavcodec-dev libavutil-dev libopencv-dev libcurl4 ros-${ROS_DISTRO}-foxglove-msgs -y
```
## GPU支持
coencoder支持使用硬件加速进行编码，支持以下编码器：
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
      "output_frame_rate": 10,
      "encode_preset": "ultrafast",
      "encode_tune": "zerolatency"
    },
    {
      "bitrate": 1600000,
      "encoder_name": "libx264",
      "input": "/camera_1/raw_image",
      "output": "/camera_1/raw_image/h264",
      "output_frame_rate": 15,
      "encode_preset": "ultrafast",
      "encode_tune": "zerolatency"
    }
  ]
}
```
* **enable_by_default:** 是否默认启用编码
* **log_directory:** 日志文件路径
* **log_level:** 日志级别，可选值：Debug / Info / Warn / Error
* **topics_param:** 数组类型，包含7个字段
  * **bitrate:** 输出码率
  * **input:** 输入topic名称
  * **output:** 输出topic名称
  * **[可选参数] encoder_name:** 编码器名称，支持`h264_nvenc`、`h264_qsv`、`h264_amf`、`h264_vaapi`，同时也可以使用`libx264`通过CPU编码帧（默认为`libx264`）
  * **[可选参数] output_frame_rate:** 输出帧率（0表示使用原始帧率）
  * **[可选参数] encode_preset:**
  
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
  * **[可选参数] encode_tune:**
  
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
    * `output_frame_rate`、`encoder_name`、`encode_preset`和`encode_tune`字段为可选字段。如果在`topics_param`中未明确指定这些字段，coEncoder将使用默认值进行h264编码

## 编译或deb安装

**您可以通过自行编译安装CoEncoder，也可以使用提供的.deb包进行安装。**
- 编译 
  - ROS1
    ```bash
    # 将项目复制到您的ROS工作空间
    cp -r ${THIS_REPO} ${YOUR_ROS_WS}/src/
    
    # Source ROS设置并构建
    source /opt/ros/${ROS_DISTRO}/setup.bash
  
    cd ${YOUR_ROS_WS}
  
    catkin_make --pkg coencoder install
    ```
  - ROS2
    ```bash
    # 将项目复制到您的ROS工作空间
    cp -r ${THIS_REPO} ${YOUR_ROS_WS}/src/
    # Source ROS设置并构建
    source /opt/ros/${ROS_DISTRO}/setup.bash
  
    cd ${YOUR_ROS_WS}
    colcon build --packages-select coencoder    
    ```
    
- deb安装
  ```bash
    dpkg -i ros-${ROS_DISTRO}-coencoder_latest_$(dpkg --print-architecture).deb
  ```

## 运行

- ROS1
  ```bash
  # 如果通过编译安装coencoder，source您的工作空间  
  source ${YOUR_ROS_WS}/install/setup.bash
  # 如果通过deb安装coencoder，source ros
  source /opt/ros/${ROS_DISTRO}/setup.bash
  
  roslaunch coencoder coencoder.launch
  # 也可使用 `rosrun` 启动节点
  rosrun coencoder coencoder --config-file ${CONFIG_FILE_PATH}
  ```
  
- ROS2
  ```bash
  # 如果通过编译安装coencoder，source您的工作空间  
  source ${YOUR_ROS_WS}/install/setup.bash
  # 如果通过deb安装coencoder，source ros
  source /opt/ros/${ROS_DISTRO}/setup.bash
  
  ros2 launch coencoder coencoder_launch.xml
  # 也可使用 `ros2 run` 启动节点  
  ros2 run coencoder coencoder -- --config-file ${CONFIG_FILE_PATH}
  ``` 