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

#include <atomic>
#include <string>
#include <memory>
#include <map>
#include <thread>
#include <sys/file.h>
#include <sys/stat.h>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
// #include <coscene_msgs/msg/compressed_video.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/set_bool.hpp>
#include <utility>

#include "utils/config.hpp"
#include "utils/encoder.hpp"
#include "utils/util.hpp"
#include "utils/json.hpp"
#include "utils/logger.hpp"
#include "utils/curl_client.hpp"

constexpr size_t DEFAULT_MIN_QOS_DEPTH = 1;
constexpr size_t DEFAULT_MAX_QOS_DEPTH = 25;

using Image = sensor_msgs::msg::Image;
using CompressedImage = sensor_msgs::msg::CompressedImage;
// using CompressedVideo = coscene_msgs::msg::CompressedVideo;
using CompressedVideo = foxglove_msgs::msg::CompressedVideo;

class CoEncoder : public rclcpp::Node {
public:
  CoEncoder()
    : Node("coencoder") {
    const char* home = std::getenv("HOME");
    if (!home) {
      COLOG_WARN("Failed to get HOME environment variable, "
        "use default config directory `/tmp/coencoder/config/config.json`");
      config_file_path_ = "/tmp/coencoder/config/config.json";
    } else {
      config_file_path_ = std::string(home) + "/.config/coencoder/config.json";
    }
    create_directory(config_file_path_);
    if (config_.load_config(config_file_path_)) {
      update_logger(config_.log_directory_, config_.log_level_);
    }

    COLOG_INFO("============================== coEncoder started ==============================");
    COLOG_INFO("config: \n%s", config_.print_config().c_str());
    update(config_);

    // Start async config update thread instead of timer to avoid blocking H264 encoding
    config_update_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !shutdown_requested_) {
        try {
          update_config_from_http();
        } catch (const std::exception& e) {
          COLOG_ERROR("Config update failed: %s", e.what());
        }
        
        // Sleep for 10 seconds, but check shutdown flag every second
        for (int i = 0; i < 10 && !shutdown_requested_; ++i) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
      }
    });

    encoder_ctrl_ = this->create_service<std_srvs::srv::SetBool>(
      "/encoder_ctrl",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        COLOG_INFO("encoder_ctrl was called");
        encoding_enabled_ = request->data;
        response->success = true;
        if (encoding_enabled_) {
          response->message = "encoder enabled";
          COLOG_INFO("encoder enabled");
        } else {
          response->message = "encoder disabled";
          COLOG_INFO("encoder disabled");
        }
        return;
      });
  }

  ~CoEncoder() override {
    // Signal shutdown and wait for config thread to finish
    shutdown_requested_ = true;
    if (config_update_thread_.joinable()) {
      config_update_thread_.join();
    }
  }

private:
  void update_config_from_http() {
    auto resp = curl_client_.get("http://127.0.0.1:22524/config/current");
    if (resp.success) {
      try {
        const nlohmann::json response_json = nlohmann::json::parse(resp.body);
        if (!response_json.contains("plugin_config")
          || !response_json["plugin_config"].contains("coEncoder")) {
          return;
        }
        const nlohmann::json encoder_config = response_json["plugin_config"]["coEncoder"];
        if (config_.update_config(encoder_config)) {
          COLOG_DEBUG("new config arrived, update with:\n%s ", encoder_config.dump(2).c_str());
          if (!config_.save_config(config_file_path_)) {
            COLOG_WARN("save config failed!");
          }
        }

        // if (encoder_config != current_config_) {
        //   COLOG_DEBUG("new config arrived, update with:\n%s ", encoder_config.dump(2).c_str());
        //   save_config(encoder_config);
        //   current_config_ = encoder_config;
        // }
        // if (check_config(encoder_config)) {
        //   update(encoder_config);
        // } else {
        //   COLOG_WARN("the response of `config/current` is invalid ");
        // }

      } catch (const nlohmann::json::parse_error & e) {
        COLOG_ERROR("Failed to parse JSON response: %s", e.what());
      }
    } else {
      COLOG_ERROR("GET request failed: %s", resp.error_message.c_str());
    }
    update(config_);
  }

  void update_logger(const std::string& log_dir, const std::string& log_lvl) {
      Logger::getInstance().set_log_dir(log_dir);
      Logger::getInstance().set_log_level(log_lvl);
  }

  void update(const Config& cfg) {
    encoding_enabled_ = cfg.enable_by_default_;

    const auto & diff = findSetsDifference(subscribed_topics_params_, cfg.topics_param);
    if (!diff.isIdentical()) {
      for (const auto & it : diff.missing) {
        removing_topic(it);
      }
      for (const auto & it : diff.added) {
        subscribe_topic(it);
      }
    }

    // const auto input_topics = encoder_config["input_topics"].get<std::vector<std::string>>();
    // const auto output_topics = encoder_config["output_topics"].get<std::vector<std::string>>();
    // for (int i = 0; i < input_topics.size(); ++i) {
    //   topics.insert(input_topics[i]);
    //   encoding_topics.emplace(input_topics[i], output_topics[i]);
    // }
    //
    // const auto & diff = findSetsDifference(subscribed_topics_, topics);
    // if (!diff.isIdentical()) {
    //   for (const auto & it : diff.added) {
    //     subscribe_topic(it, encoding_topics[it]);
    //   }
    //   for (const auto & it : diff.missing) {
    //     removing_topic(it);
    //   }
    // }
  }

  void removing_topic(const TopicParam & topic) {
    COLOG_DEBUG("removing topic [ %s ] from subscription list", topic.input_topic.c_str());
    publisher_map_.erase(topic.input_topic);
    encoder_map_.erase(topic.input_topic);
    // subscribed_topics_.erase(topic.input_topic);
    subscribed_topics_params_.erase(topic);

    if (image_sub_.find(topic.input_topic) != image_sub_.end()) {
      image_sub_.erase(topic.input_topic);
    } else if (comp_image_sub_.find(topic.input_topic) != comp_image_sub_.end()) {
      comp_image_sub_.erase(topic.input_topic);
    }
  }

  void subscribe_topic(const TopicParam & topic) {
    COLOG_DEBUG("try to subscribe topic [ %s ]", topic.input_topic.c_str());
    auto topic_names_and_types = this->get_topic_names_and_types();
    if (topic_names_and_types.empty()) {
      COLOG_DEBUG("no topic found!");
      return;
    }
    if (topic_names_and_types.find(topic.input_topic) != topic_names_and_types.end()) {
      const std::string msg_type = topic_names_and_types.find(topic.input_topic)->second[0];
      if (msg_type == "sensor_msgs/msg/Image") {
        COLOG_DEBUG("msg_type: sensor_msgs/msg/Image");
        const auto qos = get_publisher_qos(topic.input_topic);
        auto img_sub = this->create_subscription<Image>(
          topic.input_topic, qos,
          [this, topic](Image::SharedPtr msg) {
            if (encoding_enabled_) {
              if (publisher_map_.count(topic.input_topic) == 0) {
                COLOG_DEBUG("create publisher [%s]", topic.output_topic.c_str());
                const auto pub = this->create_publisher<CompressedVideo>(topic.output_topic, 10);
                publisher_map_.emplace(topic.input_topic, pub);
                encoder_map_.emplace(
                  std::piecewise_construct,
                  std::forward_as_tuple(topic.input_topic),
                  std::forward_as_tuple(msg->width, msg->height, topic.bitrate, topic.input_topic));
              }
              process_image(convertToCvMat(*msg), topic.input_topic);
            }
          });
        // subscribed_topics_.emplace(topic.input_topic);
        subscribed_topics_params_.emplace(topic);
        image_sub_.emplace(topic.input_topic, img_sub);
        COLOG_INFO("topic [ %s ] subscribed!", topic.input_topic.c_str());
      } else if (msg_type == "sensor_msgs/msg/CompressedImage") {
        COLOG_DEBUG("msg_type: sensor_msgs/msg/CompressedImage");
        const auto qos = get_publisher_qos(topic.input_topic);
        auto comp_sub = this->create_subscription<CompressedImage>(
          topic.input_topic, qos,
          [this,  topic](CompressedImage::SharedPtr msg) {
            if (encoding_enabled_) {
              const cv::Mat decoded_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
              if (decoded_img.empty()) {
                return;
              }
              if (publisher_map_.count(topic.input_topic) == 0) {
                COLOG_DEBUG("create publisher [%s]", topic.output_topic.c_str());
                const auto pub = this->create_publisher<CompressedVideo>(topic.output_topic, 10);
                publisher_map_.emplace(topic.input_topic, pub);
                encoder_map_.emplace(
                  std::piecewise_construct,
                  std::forward_as_tuple(topic.input_topic),
                  std::forward_as_tuple(decoded_img.cols, decoded_img.rows, topic.bitrate, topic.input_topic));
              }
              process_image(decoded_img, topic.input_topic);
            }
          });
        // subscribed_topics_.emplace(topic.input_topic);
        subscribed_topics_params_.emplace(topic);
        comp_image_sub_.emplace(topic.input_topic, comp_sub);
        COLOG_INFO("topic [ %s ] subscribed!", topic.input_topic.c_str());
      } else {
        COLOG_ERROR("unsupported topic type: %s", topic.input_topic.c_str());
      }
    } else {
      COLOG_WARN("topic [ %s ] not found, retry later", topic.input_topic.c_str());
    }
  }

  void process_image(cv::Mat img, const std::string & topic) {
    if (img.empty()) {
      return;
    }

    // cv::Mat img_8u;
    // if (img.depth() != CV_8U) {
    //   // Use more efficient conversion
    //   const double alpha = 255.0 / static_cast<double>(depth_image_max_val_);
    //   img.convertTo(img_8u, CV_8U, alpha);
    // } else {
    //   img_8u = img; // No copy, just reference
    // }

    // cv::Mat yuv_img;
    // const int channels = img_8u.channels();

    // if (channels == 1) {
    //   // Optimize grayscale conversion - reserve space to avoid reallocation
    //   cv::Mat bgr_img;
    //   bgr_img.reserve(img_8u.rows * img_8u.cols * 3);
    //   cv::cvtColor(img_8u, bgr_img, cv::COLOR_GRAY2BGR);
    //   cv::cvtColor(bgr_img, yuv_img, cv::COLOR_BGR2YUV_I420);
    // } else if (channels == 3) {
    //   // Direct conversion for 3-channel images
    //   cv::cvtColor(img_8u, yuv_img, cv::COLOR_BGR2YUV_I420);
    // } else {
    //   // Use throttled warning to reduce log spam
    //   COLOG_WARN("Unsupported image channels: %d", channels);
    //   return;
    // }

    // Add safety check before encoder access
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
          pub_it->second->publish(*frame);
        }
      }
    } catch (const std::exception & e) {
      COLOG_ERROR("Encoding failed for topic %s: %s", topic.c_str(), e.what());
    }
  }

  static cv::Mat convertToCvMat(const Image & img_msg) {
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

    cv::Mat image(img_msg.height, img_msg.width, cv_type, const_cast<uchar*>(img_msg.data.data()),
                  img_msg.step);
    return image;
  }

  rclcpp::QoS get_publisher_qos(const std::string & topic) {
    // Select an appropriate subscription QOS profile. This is similar to how ros2 topic echo
    // does it:
    // https://github.com/ros2/ros2cli/blob/619b3d1c9/ros2topic/ros2topic/verb/echo.py#L137-L194
    size_t depth = 0;
    size_t reliability_reliable_endpoints_count = 0;
    size_t durability_transient_local_endpoints_count = 0;

    const auto publisher_info = this->get_publishers_info_by_topic(topic);
    COLOG_DEBUG("topic %s has %zu publishers", topic.c_str(), publisher_info.size());
    
    for (const auto & publisher : publisher_info) {
      const auto & qos = publisher.qos_profile();

      COLOG_DEBUG("  publisher: [%s], depth=%zu, reliability=%s, durability=%s",
                 publisher.node_name().c_str(),
                 qos.get_rmw_qos_profile().depth,
                 (qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) ? "RELIABLE" : "BEST_EFFORT",
                 (qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) ? "TRANSIENT_LOCAL" : "VOLATILE");

      if (qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
        ++reliability_reliable_endpoints_count;
      }
      if (qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
        ++durability_transient_local_endpoints_count;
      }
      const size_t publisher_history_depth = std::max(1ul, qos.get_rmw_qos_profile().depth);
      depth = depth + publisher_history_depth;
    }

    depth = std::max(depth, DEFAULT_MIN_QOS_DEPTH);
    if (depth > DEFAULT_MAX_QOS_DEPTH) {
      COLOG_WARN(
        "Limiting history depth for topic '%s' to %zu (was %zu). You may want to increase "
        "the max_qos_depth parameter value.",
        topic.c_str(), DEFAULT_MAX_QOS_DEPTH, depth);
      depth = DEFAULT_MAX_QOS_DEPTH;
    }

    rclcpp::QoS qos{rclcpp::KeepLast(depth)};

    // If all endpoints are reliable, ask for reliable
    if (reliability_reliable_endpoints_count == publisher_info.size()) {
      qos.reliable();
    } else {
      if (reliability_reliable_endpoints_count > 0) {
        COLOG_WARN(
          "Some, but not all, publishers on topic '%s' are offering QoSReliabilityPolicy.RELIABLE. "
          "Falling back to QoSReliabilityPolicy.BEST_EFFORT as it will connect to all publishers",
          topic.c_str());
      }
      qos.best_effort();
    }

    // If all endpoints are transient_local, ask for transient_local
    if (durability_transient_local_endpoints_count == publisher_info.size()) {
      qos.transient_local();
    } else {
      if (durability_transient_local_endpoints_count > 0) {
        COLOG_WARN(
          "Some, but not all, publishers on topic '%s' are offering "
          "QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to "
          "QoSDurabilityPolicy.VOLATILE as it will connect to all publishers",
          topic.c_str());
      }
      qos.durability_volatile();
    }
    
    COLOG_DEBUG("  use QoS[depth=%zu, reliability=%s, durability=%s] to subscribe topic: %s",
               qos.get_rmw_qos_profile().depth,
               (qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) ? "RELIABLE" : "BEST_EFFORT",
               (qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) ? "TRANSIENT_LOCAL" : "VOLATILE",
               topic.c_str());
    
    return qos;
  }

  static std::string format_topics(const std::vector<std::string> & topics) {
    std::string result;
    for (const auto & topic : topics) {
      result += "'" + topic + "' ";
    }
    return result;
  }

  static std::string trim(const std::string & str) {
    size_t first = str.find_first_not_of(' ');
    if (std::string::npos == first) {
      return str;
    }
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
  }

  bool get_image_size(const std::string & resolution, int & width, int & height) {
    std::string trim_str = trim(resolution);
    size_t pos = trim_str.find('*');
    if (pos == std::string::npos) {
      return false;
    }

    try {
      width = std::stoi(trim_str.substr(0, pos));
      height = std::stoi(trim_str.substr(pos + 1));
      return true;
    } catch (const std::invalid_argument & e) {
      COLOG_ERROR("invalid_argument: %s", e.what());
      return false;
    } catch (const std::out_of_range & e) {
      COLOG_ERROR("out_of_range: %s", e.what());
      return false;
    }
  }


  CurlClient curl_client_;
  Config config_;

  std::string config_file_path_ = "/tmp/coencoder/config";
  std::string log_directory_ = "/tmp/coencoder/log/";
  std::string log_level_ = "Info";

  // std::set<std::string> subscribed_topics_;
  std::set<TopicParam> subscribed_topics_params_;

  // nlohmann::json current_config_{};
  std::atomic<bool> encoding_enabled_{true};
  std::atomic<bool> shutdown_requested_{false};

  std::map<std::string, std::shared_ptr<rclcpp::Subscription<Image>>> image_sub_;
  std::map<std::string, std::shared_ptr<rclcpp::Subscription<CompressedImage>>> comp_image_sub_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<CompressedVideo>>> publisher_map_;
  std::map<std::string, H264Encoder> encoder_map_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr encoder_ctrl_;
  std::thread config_update_thread_;

  int depth_image_max_val_ = 10000;;

  // rclcpp::TimerBase::SharedPtr update_config_timer_; // Removed as per edit hint
};

#endif //  COENCODER_H
