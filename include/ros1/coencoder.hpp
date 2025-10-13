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

#ifndef ROS1__COENCODER_HPP_
#define ROS1__COENCODER_HPP_

#include <chrono>
#include <fstream>
#include <map>
#include <set>
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
// #include <coscene_msgs/CompressedVideo.h>
// #include <foxglove_msgs/CompressedVideo.h>
#include <std_srvs/SetBool.h>
#include <opencv2/opencv.hpp>

#include "json.hpp"

#include "utils/encoder.hpp"
#include "utils/curl_client.hpp"
#include "utils/logger.hpp"
#include "utils/config.hpp"
#include "utils/thread_pool.hpp"

class CoEncoder
{
public:
  explicit CoEncoder(const std::string & config_file)
  : nh_("~"), thread_pool_(1)
  {
    const char * home = std::getenv("HOME");
    if (config_file.empty()) {
      if (!home) {
        COLOG_WARN(
          "Failed to get HOME environment variable, "
          "use default config directory `/tmp/coencoder/config/config.json`");
        config_file_path_ = "/tmp/coencoder/config/config.json";
      } else {
        config_file_path_ = std::string(home) + "/.config/coencoder/config.json";
      }
    } else {
      config_file_path_ = config_file;
    }
    create_directory(config_file_path_);
    config_.load_config(config_file_path_);

    size_t initial_thread_count = config_.topics_param.size();
    if (initial_thread_count > 0) {
      thread_pool_.resize(initial_thread_count);
      COLOG_INFO(
        "Initialized thread pool with %zu threads for %zu topics", initial_thread_count,
        initial_thread_count);
    }

    update_logger(config_.log_directory_, config_.log_level_);

    COLOG_INFO("============================== coEncoder started ==============================");
    COLOG_INFO("config: \n%s", config_.print_config().c_str());
    update(config_);


    update_config_timer_ = nh_.createTimer(
      ros::Duration(10), &CoEncoder::update_config_callback,
      this);

    encoder_ctrl_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
      "encoder_ctrl", [this](std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res) {
        COLOG_INFO("encoder_ctrl was called");
        encoding_enabled_ = req.data;
        res.success = true;
        if (encoding_enabled_) {
          res.message = "encoder enabled";
          COLOG_INFO("encoder enabled");
        } else {
          res.message = "encoder disabled";
          COLOG_INFO("encoder disabled");
        }
        return true;
      });
  }

  ~CoEncoder() = default;

private:
  void update_logger(const std::string & log_dir, const std::string & log_lvl)
  {
    Logger::getInstance().set_log_dir(log_dir);
    Logger::getInstance().set_log_level(log_lvl);
  }

  void update(const Config & cfg)
  {
    encoding_enabled_ = cfg.enable_by_default_;

    // 动态调整线程池大小，根据配置中的topic数量
    size_t new_thread_count = cfg.topics_param.size();
    if (new_thread_count > 0) {
      size_t current_thread_count = thread_pool_.get_thread_count();
      if (current_thread_count != new_thread_count) {
        COLOG_INFO(
          "Resizing thread pool from %zu to %zu threads", current_thread_count,
          new_thread_count);
        thread_pool_.resize(new_thread_count);
      }
    }

    const auto & diff = findSetsDifference(subscribed_topics_params_, cfg.topics_param);
    if (!diff.isIdentical()) {
      for (const auto & it : diff.missing) {
        removing_topic(it);
      }
      for (const auto & it : diff.added) {
        subscribe_topic(it);
      }
    }
  }

  void update_config_callback(const ros::TimerEvent &)
  {
    auto resp = curl_client_.get("http://127.0.0.1:22524/config/current");
    if (resp.success) {
      try {
        const nlohmann::json response_json = nlohmann::json::parse(resp.body);
        if (!response_json.contains("plugin_config") ||
          !response_json["plugin_config"].contains("coEncoder"))
        {
          return;
        }
        const nlohmann::json encoder_config = response_json["plugin_config"]["coEncoder"];

        if (config_.update_config(encoder_config)) {
          COLOG_INFO("new config arrived, update with:\n%s ", encoder_config.dump(2).c_str());
          update_logger(config_.log_directory_, config_.log_level_);
          update(config_);
          config_.save_config(config_file_path_);
        }
      } catch (const nlohmann::json::parse_error & e) {
        COLOG_ERROR("Failed to parse JSON response: %s", e.what());
      }
    } else {
      COLOG_ERROR("GET request failed: %s", resp.error_message.c_str());
    }
  }

  void subscribe_topic(const TopicParam & topic)
  {
    COLOG_INFO(
      "Subscribing to topic: %s, publish topic: %s", topic.input_topic.c_str(),
      topic.output_topic.c_str());
    const std::string topic_type = get_topic_type(topic.input_topic);
    if (topic_type == "sensor_msgs/Image") {
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::Image>(
        topic.input_topic, 1,
        [this, topic](const sensor_msgs::Image::ConstPtr & msg) {
          if (encoding_enabled_) {
            if (encoder_map_.count(topic.input_topic) == 0) {
              COLOG_INFO("create encoder [%s]", topic.output_topic.c_str());
              encoder_map_.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(topic.input_topic),
                std::forward_as_tuple(
                  msg->width, msg->height, topic.bitrate, topic.encoder_name));
            }
            process_image(
              convertToCvMat(*msg), topic.input_topic,
              static_cast<int64_t>(msg->header.stamp.sec * 1e9 + msg->header.stamp.nsec));
          }
        });
      subscriber_map_.emplace(topic.input_topic, sub);
      subscribed_topics_params_.emplace(topic);
      COLOG_INFO("topic [ %s ] subscribed!", topic.input_topic.c_str());
      if (publisher_map_.count(topic.input_topic) == 0) {
        COLOG_INFO("create publisher [%s]", topic.output_topic.c_str());
        ros::Publisher pub = nh_.advertise<CompressedVideo>(topic.output_topic, 1);
        publisher_map_.emplace(topic.input_topic, pub);
      }
    } else if (topic_type == "sensor_msgs/CompressedImage") {
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::CompressedImage>(
        topic.input_topic, 1,
        [this, topic](const sensor_msgs::CompressedImage::ConstPtr & msg) {
          if (encoding_enabled_) {
            const cv::Mat decoded_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
            if (encoder_map_.count(topic.input_topic) == 0) {
              COLOG_INFO("create encoder [%s]", topic.output_topic.c_str());
              encoder_map_.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(topic.input_topic),
                std::forward_as_tuple(
                  decoded_img.cols, decoded_img.rows, topic.bitrate, topic.encoder_name));
            }
            process_image(
              decoded_img, topic.input_topic,
              static_cast<int64_t>(msg->header.stamp.sec * 1e9 + msg->header.stamp.nsec));
          }
        });
      subscriber_map_.emplace(topic.input_topic, sub);
      subscribed_topics_params_.emplace(topic);
      COLOG_INFO("topic [ %s ] subscribed!", topic.input_topic.c_str());
      if (publisher_map_.count(topic.input_topic) == 0) {
        COLOG_INFO("create publisher [%s]", topic.output_topic.c_str());
        ros::Publisher pub = nh_.advertise<CompressedVideo>(topic.output_topic, 1);
        publisher_map_.emplace(topic.input_topic, pub);
      }
    } else {
      COLOG_INFO(
        "Unsupported message type [%s] for topic '%s'", topic_type.c_str(),
        topic.input_topic.c_str());
    }
  }

  void removing_topic(const TopicParam & topic)
  {
    COLOG_DEBUG("removing topic [ %s ] from subscription list", topic.input_topic.c_str());
    publisher_map_.erase(topic.input_topic);

    COLOG_DEBUG("destruct encoder of topic [ %s ]", topic.input_topic.c_str());
    encoder_map_.erase(topic.input_topic);
    subscriber_map_.erase(topic.input_topic);

    subscribed_topics_params_.erase(topic);
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
    return image;
  }

  void process_image(const cv::Mat & img, const std::string & topic, const int64_t & timestamp)
  {
    if (img.empty()) {
      COLOG_WARN("Empty image received");
      return;
    }

    // 提交到线程池异步处理，避免阻塞主线程
    thread_pool_.enqueue(
      [this, img, topic, timestamp]() {
        process_image_single(img, topic, timestamp);
      });
  }

  void process_image_single(
    const cv::Mat & img, const std::string & topic,
    const int64_t & timestamp)
  {
    auto encoder_it = encoder_map_.find(topic);
    if (encoder_it == encoder_map_.end()) {
      COLOG_WARN("Encoder not found for topic: %s", topic.c_str());
      return;
    }
    try {
      // Send frame to encoder
      encoder_it->second.send_frame(img, timestamp);

      // Try to retrieve encoded frame (may return nullptr if encoder needs more input)
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
  Config config_;

  std::string config_file_path_ = "/tmp/coencoder/config";

  std::atomic<bool> encoding_enabled_{true};

  std::set<TopicParam> subscribed_topics_params_;

  std::map<std::string, ros::Subscriber> subscriber_map_;
  std::map<std::string, ros::Publisher> publisher_map_;

  ros::ServiceServer encoder_ctrl_;

  std::map<std::string, H264Encoder> encoder_map_;

  int bitrate_ = 800000, depth_image_max_val_ = 10000;

  ros::Timer update_config_timer_;

  ThreadPool thread_pool_;
};

#endif  // ROS1__COENCODER_HPP_
