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
#include "singleton_lock.hpp"

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

    pending_topics_ = sub_topics_;
    for (size_t i = 0; i < sub_topics_.size(); ++i) {
      const std::string & topic = sub_topics_[i];
      const std::string & resolution = resolutions_[i];
      int width, height;
      if (!get_image_size(resolution, width, height)) {
        ROS_INFO("resolution format error: '%s'", resolution.c_str());
        continue;
      }
      topic_resolution_[topic] = std::make_pair(width, height);
    }

    retry_timer_ = nh_.createTimer(ros::Duration(5.0), &CoEncoder::retry_subscribe_topics, this);

    retry_subscribe_topics(ros::TimerEvent());

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


  void retry_subscribe_topics(const ros::TimerEvent&)
  {
    if (pending_topics_.empty()) {
      retry_timer_.stop();
      ROS_INFO("All topics subscribed successfully, stop retry timer.");
      return;
    }

    get_all_topics_and_type();

    for (auto it = pending_topics_.begin(); it != pending_topics_.end(); ) {
      const std::string& topic = *it;
      auto topic_it = topic_map_.find(topic);

      if (topic_it == topic_map_.end()) {
        ROS_DEBUG("Topic %s not found in ROS master, will retry.", topic.c_str());
        ++it;
        continue;
      }

      std::string topic_type = topic_it->second;
      if (topic_type != "sensor_msgs/Image" && topic_type != "sensor_msgs/CompressedImage") {
        ROS_WARN("Unsupported message type '%s' for topic '%s', will retry", topic_type.c_str(), topic.c_str());
        ++it;
        continue;
      }

      if (std::find_if(subscribers_.begin(), subscribers_.end(), 
          [&topic](const ros::Subscriber& s) { return s.getTopic() == topic; }) != subscribers_.end()) {
        ROS_DEBUG("Topic %s already subscribed, removing from pending list.", topic.c_str());
        it = pending_topics_.erase(it);
        continue;
      }

      auto resolution_it = topic_resolution_.find(topic);
      if (resolution_it == topic_resolution_.end()) {
        ROS_WARN("No resolution found for topic %s, will retry", topic.c_str());
        ++it;
        continue;
      }

      const auto& resolution = resolution_it->second;
      std::string pub_topic = topic + "/h264";

      if (publisher_map_.count(pub_topic) == 0) {
        ros::Publisher pub = nh_.advertise<CompressedVideo>(pub_topic, 1);
        publisher_map_.emplace(pub_topic, pub);
      }

      if (encoder_map_.count(pub_topic) == 0) {
        encoder_map_.emplace(
          std::piecewise_construct,
          std::forward_as_tuple(pub_topic),
          std::forward_as_tuple(resolution.first, resolution.second, bitrate_, output_fps_));
      }

      ros::Subscriber sub;
      bool subscribe_success = false;

      if (topic_type == "sensor_msgs/Image") {
        sub = nh_.subscribe<sensor_msgs::Image>(
          topic, 1,
          [this, pub_topic](const sensor_msgs::Image::ConstPtr& msg)
          {
            if (encoding_enabled_) {
              process_image(convertToCvMat(*msg), pub_topic);
            }
          });
        subscribe_success = true;
      } else if (topic_type == "sensor_msgs/CompressedImage") {
        sub = nh_.subscribe<sensor_msgs::CompressedImage>(
          topic, 1,
          [this, pub_topic](const sensor_msgs::CompressedImage::ConstPtr& msg)
          {
            if (encoding_enabled_) {
              cv::Mat decoded_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
              process_image(decoded_img, pub_topic);
            }
          });
        subscribe_success = true;
      }

      if (subscribe_success) {
        subscribers_.emplace_back(sub);

        if (timer_map_.count(pub_topic) == 0) {
          auto timer = nh_.createTimer(
            ros::Duration(1.0 / static_cast<double>(output_fps_)),
            [this, pub_topic](const ros::TimerEvent&)
            {
              if (encoding_enabled_) {
                auto frame = encoder_map_[pub_topic].encode_frame();
                if (frame) {
                  publisher_map_[pub_topic].publish(*frame);
                }
              }
            }, false, true);
          timer_map_.emplace(pub_topic, timer);
        }

        ROS_INFO("Successfully subscribed topic: %s (type: %s)", topic.c_str(), topic_type.c_str());
        it = pending_topics_.erase(it);
      } else {
        ROS_WARN("Failed to subscribe topic: %s, will retry", topic.c_str());
        ++it;
      }
    }
  }

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

  void process_image(cv::Mat img, const std::string & pub_topic)
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
    topic_map_.clear();
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

  std::unordered_map<std::string, std::pair<int, int>> topic_resolution_;

  std::map<std::string, std::string> topic_map_;

  int output_fps_ = 30, bitrate_ = 800000, depth_image_max_val_ = 10000;

  std::vector<std::string> pending_topics_;
  ros::Timer retry_timer_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "coencoder");

  coscene::SingletonLock lock("coencoder");
  if (!lock.acquire()) {
    ROS_ERROR("Failed to acquire singleton lock. Another instance may be running.");
    return 1;
  }

  lock.setup_signal_handlers([](){ ros::shutdown(); });

  try {
    CoEncoder node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in main: %s", e.what());
  }
  return 0;
}
