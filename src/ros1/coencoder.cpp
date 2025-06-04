// Copyright 2024 coScene
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <thread>
#include <chrono>
#include <vector>
#include <map>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <foxglove_msgs/CompressedVideo.h>
#include <std_srvs/SetBool.h>
#include <opencv2/opencv.hpp>
#include "encoder.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#define TARGET_WIDTH 640

class CoEncoder
{
public:
  CoEncoder()
  : nh_("~")
  {
    nh_.param("output_fps", output_fps_, 30);
    nh_.param("bitrate", bitrate_, 800000);
    nh_.param("depth_image_max_value", depth_image_max_val_, 1000);
    ROS_INFO(
      "[ros1 constructor] output_fps: %d, bitrate: %d, depth_image_max_val: %d", output_fps_,
      bitrate_, depth_image_max_val_);

    if (!nh_.getParam("subscribe_topics", sub_topics_)) {
      ROS_ERROR("Failed to get param 'subscribe_topics'");
      ros::shutdown();
      exit(-1);
    }

    if (!nh_.getParam("video_resolutions", resolutions_)) {
      ROS_ERROR("Failed to get param 'video_resolutions'");
      ros::shutdown();
      exit(-1);
    }
    ROS_INFO("[constructor] sub_topics: %s", format_topics(sub_topics_).c_str());

    get_all_topics_and_type();
    ros::Duration interval(1.0 / static_cast<double>(output_fps_));

    for (size_t i = 0; i < sub_topics_.size(); ++i) {
      const std::string & topic = sub_topics_[i];
      const std::string & resolution = resolutions_[i];

      std::string msg_type = topic_map_[topic];
      if (msg_type != "sensor_msgs/CompressedImage" && msg_type != "sensor_msgs/Image") {
        ROS_WARN("Unsupported message type '%s' for topic '%s'", msg_type.c_str(), topic.c_str());
        continue;
      }

      int width = 0, height = 0;
      if (!get_image_size(resolution, width, height)) {
        ROS_WARN(
          "Failed to parse resolution '%s' for topic: %s", resolution.c_str(),
          topic.c_str());
        continue;
      }
      ROS_INFO("topic: %s, image size : %d*%d", topic.c_str(), width, height);

      std::string pub_topic = topic + "/h264";
      encoder_map_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(pub_topic),
        std::forward_as_tuple(width, height, bitrate_, output_fps_));
      ros::Publisher pub = nh_.advertise<CompressedVideo>(pub_topic, 1);
      ros::Subscriber sub;
      if (msg_type == "sensor_msgs/CompressedImage") {
        sub = nh_.subscribe<sensor_msgs::CompressedImage>(
          topic, 1,
          [this, pub_topic, height](const sensor_msgs::CompressedImage::ConstPtr & msg)
          {
            if (encoding_enabled_) {
              cv::Mat decoded_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
              process_image(decoded_img, pub_topic, height);
            }
          });
      } else {
        sub = nh_.subscribe<sensor_msgs::Image>(
          topic, 1,
          [this, pub_topic, height](const sensor_msgs::Image::ConstPtr & msg)
          {
            if (encoding_enabled_) {
              process_image(convertToCvMat(*msg), pub_topic, height);
            }
          });
      }
      subscribers_.emplace_back(sub);
      publisher_map_.emplace(pub_topic, pub);
    }

    for (const auto & pub_topic : sub_topics_) {
      if (timer_map_.find(pub_topic) == timer_map_.end()) {
        auto timer = nh_.createTimer(
          interval,
          [this, pub_topic](const ros::TimerEvent &)
          {
            if (encoding_enabled_) {
              auto frame = encoder_map_[pub_topic + "/h264"].
              encode_frame();
              if (frame) {
                publisher_map_[pub_topic + "/h264"].publish(*frame);
              }
            }
          }, false, true);
        timer_map_.emplace(pub_topic, timer);
      }
    }

    encoder_ctrl_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
      "encoder_ctrl", [this](std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
      {
        ROS_INFO("encoder_ctrl was called");
        encoding_enabled_ = req.data;
        res.success = true;
        if (encoding_enabled_) {
          res.message = "encoder enabled";
          ROS_INFO("encoder enabled");
        } else {
          res.message = "encoder disabled";
          ROS_INFO("encoder disabled");
        }
        return true;
      });
  }

  ~CoEncoder() = default;

private:
  static cv::Mat convertToCvMat(const sensor_msgs::Image & img_msg)
  {
    int cv_type = CV_8UC3;
    std::string encoding = img_msg.encoding;
    if (encoding == "bgr8" || encoding == "rgb8") {
      cv_type = CV_8UC3;
    } else if (encoding == "bgra8" || encoding == "rgba8") {
      cv_type = CV_8UC4;
    } else if (encoding == "mono8") {
      cv_type = CV_8UC1;
    } else if (encoding == "16UC1") {
      cv_type = CV_16UC1;
    } else {
      throw std::runtime_error("Unsupported encoding type: " + encoding);
    }

    cv::Mat image(img_msg.height, img_msg.width, cv_type, const_cast<uchar *>(img_msg.data.data()),
      img_msg.step);
    if (img_msg.encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }
    return image;
  }


  void process_image(cv::Mat img, const std::string & pub_topic, int height)
  {
    if (img.empty()) {
      ROS_WARN("Empty image received");
      return;
    }

    cv::Mat img_8u;
    if (img.depth() != CV_8U) {
      double alpha = 255.0 / static_cast<float>(depth_image_max_val_);
      img.convertTo(img_8u, CV_8U, alpha);
    } else {
      img_8u = img;
    }

    cv::Mat yuv_img;
    if (img_8u.channels() == 1) {
      cv::Mat bgr_img;
      cv::cvtColor(img_8u, bgr_img, cv::COLOR_GRAY2BGR);
      cv::cvtColor(bgr_img, yuv_img, cv::COLOR_BGR2YUV_I420);
    } else if (img.channels() == 3) {
      cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);
    } else {
      ROS_WARN("Unsupported image channels: %d", img.channels());
      return;
    }

    encoder_map_[pub_topic].send_frame(yuv_img);
  }

  static std::string format_topics(const std::vector<std::string> & topics)
  {
    std::string result;
    for (const auto & topic : topics) {
      result += "'" + topic + "' ";
    }
    return result;
  }

  void get_all_topics_and_type()
  {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (const auto & topic : topics) {
      topic_map_.emplace(topic.name, topic.datatype);
    }
  }

  static bool get_image_size(const std::string & resolution, int & width, int & height)
  {
    std::string trim_str = trim(resolution);
    size_t pos = trim_str.find('*');
    if (pos == std::string::npos) {
      ROS_WARN("Illegal resolution format: %s", trim_str.c_str());
      return false;
    }

    try {
      width = std::stoi(trim_str.substr(0, pos));
      height = std::stoi(trim_str.substr(pos + 1));
      return true;
    } catch (const std::invalid_argument & e) {
      ROS_WARN("invalid_argument: %s", e.what());
      return false;
    } catch (const std::out_of_range & e) {
      ROS_WARN("out_of_range: %s", e.what());
      return false;
    }
  }

  static std::string trim(const std::string & str)
  {
    size_t first = str.find_first_not_of(' ');
    if (std::string::npos == first) {
      return str;
    }
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
  }

  ros::NodeHandle nh_;

  std::atomic<bool> encoding_enabled_{true};

  std::vector<std::string> sub_topics_;
  std::vector<std::string> resolutions_;

  std::vector<ros::Subscriber> subscribers_;
  std::map<std::string, ros::Publisher> publisher_map_;

  ros::ServiceServer encoder_ctrl_;

  std::map<std::string, H264Encoder> encoder_map_;
  std::map<std::string, ros::Timer> timer_map_;

  std::map<std::string, std::string> topic_map_;

  int output_fps_ = 30, bitrate_ = 800000, depth_image_max_val_ = 10000;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "coencoder");
  CoEncoder node;
  ros::spin();
  return 0;
}
