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

#ifndef COENCODER_H
#define COENCODER_H

#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <coscene_msgs/CompressedVideo.h>
#include <std_srvs/SetBool.h>
#include <opencv2/opencv.hpp>

#include "utils/encoder.hpp"
#include "utils/curl_client.hpp"
#include "utils/logger.hpp"
#include "utils/json.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class CoEncoder
{
public:
  CoEncoder()
  : nh_("~")
  {
    const char* home = std::getenv("HOME");
    if (!home) {
      ROS_WARN("Failed to get HOME environment variable, use default config directory `/tmp/coencoder/config/config.json`");
      config_file_path_ = "/tmp/coencoder/config/config.json";
    } else {
      config_file_path_ = std::string(home) + "/.config/coencoder/config.json";
    }
    create_directory(config_file_path_);
    load_config();

    update_config_timer_ = nh_.createTimer(ros::Duration(30), &CoEncoder::update_config_callback, this);

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
  bool load_config()
  {
    if (access(config_file_path_.c_str(), F_OK) == -1) {
      ROS_INFO("Config file not found at %s, using default config", config_file_path_.c_str());
      return false;
    }

    try {
      std::ifstream config_file(config_file_path_);
      if (!config_file.is_open()) {
        ROS_WARN("Failed to open config file %s", config_file_path_.c_str());
        return false;
      }

      nlohmann::json config_json;
      config_file >> config_json;
      config_file.close();

      if (!check_config(config_json)) {
        return false;
      }
      update(config_json);
      return true;
    } catch (const nlohmann::json::parse_error& e) {
      ROS_ERROR("Failed to parse config file %s: %s", config_file_path_.c_str(), e.what());
      return false;
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to load config file %s: %s", config_file_path_.c_str(), e.what());
      return false;
    }
  }

  bool save_config(const nlohmann::json & config_json)
  {
    std::ofstream config_file(config_file_path_);
    if (!config_file.is_open()) {
      ROS_WARN("Failed to open config file for writing: %s", config_file_path_.c_str());
      return false;
    }

    config_file << config_json.dump(2);
    config_file.close();

    ROS_INFO("Successfully saved config to %s", config_file_path_.c_str());
    return true;
  }

  void update(nlohmann::json& encoder_config)
  {
    bool updated = false;
    std::set<std::string> topics;
    std::map<std::string, std::string> encoding_topics;
    if (encoder_config.contains("log_directory")) {
      const auto log_dir_in_config =encoder_config["log_directory"].get<std::string>();
      if (log_dir_in_config != log_directory_) {
        updated = true;
        log_directory_ = log_dir_in_config;
        Logger::getInstance().set_log_dir(log_directory_);
      }
    }
    if (encoder_config.contains("log_level")) {
      const auto log_level_in_config = encoder_config["log_level"].get<std::string>();
      if (log_level_in_config != log_level_) {
        updated = true;
        log_level_ = log_level_in_config;
        Logger::getInstance().set_log_level(log_level_);
      }
    }

    const auto input_topics = encoder_config["input_topics"].get<std::vector<std::string>>();
    const auto output_topics = encoder_config["output_topics"].get<std::vector<std::string>>();
    for (int i = 0; i < input_topics.size(); ++i) {
      topics.insert(input_topics[i]);
      encoding_topics.emplace(input_topics[i], output_topics[i]);
    }

    const auto& diff = findSetsDifference(subscribed_topics_, topics);
    if (!diff.isIdentical()) {
      updated = true;
      for (const auto& it : diff.added) {
        subscribe_topic(it, encoding_topics[it]);
      }
      for (const auto& it : diff.missing) {
        removing_topic(it);
      }
    }
    if (updated) {
      save_config(encoder_config);
    }
  }

  void update_config_callback(const ros::TimerEvent& )
  {
    auto resp = curl_client_.get("http://127.0.0.1:22524/config/current");
    if (resp.success) {
      try {
        nlohmann::json response_json = nlohmann::json::parse(resp.body);
        if (!response_json.contains("plugin_config")
          || !response_json["plugin_config"].contains("coEncoder")) {
          return;
        }
        nlohmann::json encoder_config = response_json["plugin_config"]["coEncoder"];
        if (check_config(encoder_config)) {
          update(encoder_config);
        } else {
          ROS_WARN("the response of `config/current` is invalid ");
        }
      } catch (const nlohmann::json::parse_error& e) {
        ROS_ERROR("Failed to parse JSON response: %s", e.what());
      }
    } else {
      ROS_ERROR("GET request failed: %s", resp.error_message.c_str());
    }
  }

  bool check_config(nlohmann::json& json_obj)
  {
    if (!json_obj.contains("input_topics")) {
      ROS_DEBUG("'plugin_config.coEncoder.input_topics' not found in response of config/current");
      return false;
    }
    if (!json_obj.contains("output_topics")) {
      ROS_DEBUG("'plugin_config.coEncoder.output_topics' not found in response of config/current");
      return false;
    }

    const auto& input_topics = json_obj["input_topics"];
    const auto& output_topics = json_obj["output_topics"];

    if (!input_topics.is_array()) {
      ROS_DEBUG("'plugin_config.coEncoder.input_topics' is not an array");
      return false;
    }
    
    if (!output_topics.is_array()) {
      ROS_DEBUG("'plugin_config.coEncoder.output_topics' is not an array");
      return false;
    }

    if (input_topics.size() != output_topics.size()) {
      ROS_DEBUG("'plugin_config.coEncoder.input_topics' and 'plugin_config.coEncoder.output_topics' must have same count");
      return false;
    }

    return true;
  }

  void subscribe_topic(std::string sub_topic, std::string pub_topic)
  {
    ROS_INFO("Subscribing to topic: %s, publish topic: %s", sub_topic.c_str(), pub_topic.c_str());
    const std::string topic_type = get_topic_type(sub_topic);
    if (topic_type == "sensor_msgs/Image") {
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::Image>(
        sub_topic, 1,
        [this, sub_topic, pub_topic](const sensor_msgs::Image::ConstPtr & msg)
        {
          if (encoding_enabled_) {
            if (publisher_map_.count(sub_topic) == 0) {
              ros::Publisher pub = nh_.advertise<CompressedVideo>(pub_topic, 1);
              publisher_map_.emplace(sub_topic, pub);
              encoder_map_.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(sub_topic),
                std::forward_as_tuple(msg->width, msg->height, bitrate_));
            }
            process_image(convertToCvMat(*msg), sub_topic);
          }
        });
      subscriber_map_.emplace(sub_topic, sub);
      subscribed_topics_.emplace(sub_topic);
    } else if (topic_type == "sensor_msgs/CompressedImage") {
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::CompressedImage>(
        sub_topic, 1,
        [this, sub_topic, pub_topic](const sensor_msgs::CompressedImage::ConstPtr & msg)
        {
          if (encoding_enabled_) {
            const cv::Mat decoded_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
            if (publisher_map_.count(sub_topic) == 0) {
              ros::Publisher pub = nh_.advertise<CompressedVideo>(pub_topic, 1);
              publisher_map_.emplace(sub_topic, pub);
              encoder_map_.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(sub_topic),
                std::forward_as_tuple(decoded_img.cols, decoded_img.rows, bitrate_));
            }
            process_image(decoded_img, sub_topic);
          }
        });
      subscriber_map_.emplace(sub_topic, sub);
      subscribed_topics_.emplace(sub_topic);
    } else {
      ROS_WARN("Unsupported message type [%s] for topic '%s'", topic_type.c_str(), sub_topic.c_str());
    }
  }

  void removing_topic(std::string topic)
  {
    publisher_map_.erase(topic);
    subscriber_map_.erase(topic);
    encoder_map_.erase(topic);
    subscribed_topics_.erase(topic);
  }

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
    // if (img_msg.encoding == sensor_msgs::image_encodings::RGB8) {
    //   cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    // }
    return image;
  }

  void process_image(cv::Mat img, const std::string & topic)
  {
    if (img.empty()) {
      ROS_WARN("Empty image received");
      return;
    }

    // cv::Mat img_8u;
    // if (img.depth() != CV_8U) {
    //   double alpha = 255.0 / static_cast<float>(depth_image_max_val_);
    //   img.convertTo(img_8u, CV_8U, alpha);
    // } else {
    //   img_8u = img;
    // }

    // cv::Mat yuv_img;
    // if (img_8u.channels() == 1) {
    //   cv::Mat bgr_img;
    //   cv::cvtColor(img_8u, bgr_img, cv::COLOR_GRAY2BGR);
    //   cv::cvtColor(bgr_img, yuv_img, cv::COLOR_BGR2YUV_I420);
    // } else if (img.channels() == 3) {
    //   cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);
    // } else {
    //   ROS_WARN("Unsupported image channels: %d", img.channels());
    //   return;
    // }

    auto encoder_it = encoder_map_.find(topic);
    if (encoder_it == encoder_map_.end()) {
      COLOG_WARN("Encoder not found for topic: %s", topic.c_str());
      return;
    }
    // Add error handling for encoding
    try {
      // encoder_it->second.send_frame(yuv_img);
      encoder_it->second.send_frame(img);
      const auto frame = encoder_it->second.encode_frame();
      if (frame) {
        auto pub_it = publisher_map_.find(topic);
        if (pub_it != publisher_map_.end()) {
          pub_it->second.publish(*frame);
        }
      }
    } catch (const std::exception & e) {
      COLOG_ERROR("Encoding failed for topic %s: %s", topic.c_str(), e.what());
    }
  }

  std::string get_topic_type(const std::string & topic_name)
  {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (const auto & topic : topics) {
      if (topic.name == topic_name) {
        return topic.datatype;
      }
    }
    return "";
  }

  ros::NodeHandle nh_;
  CurlClient curl_client_;

  std::string config_file_path_ = "/tmp/coencoder/config";
  std::string log_directory_ = "/tmp/coencoder/log/";
  std::string log_level_ = "Info";

  std::atomic<bool> encoding_enabled_{true};

  std::vector<std::string> input_topics_;
  std::vector<std::string> output_topics_;

  std::set<std::string> subscribed_topics_;
  std::map<std::string, ros::Subscriber> subscriber_map_;
  std::map<std::string, ros::Publisher> publisher_map_;

  ros::ServiceServer encoder_ctrl_;

  std::map<std::string, H264Encoder> encoder_map_;

  int bitrate_ = 800000, depth_image_max_val_ = 10000;

  ros::Timer update_config_timer_;
};

#endif //COENCODER_H
