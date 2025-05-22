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

#include <atomic>
#include <string>
#include <memory>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/set_bool.hpp>
#include "encoder.hpp"

class CoEncoder : public rclcpp::Node
{
public:
  CoEncoder()
  : Node("coencoder")
  {
    this->declare_parameter("output_fps", 30);
    this->declare_parameter("bitrate", 800000);
    this->declare_parameter<std::vector<std::string>>(
      "subscribe_topics",
      std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>(
      "video_resolutions",
      std::vector<std::string>{});

    this->get_parameter_or("output_fps", output_fps_, 30);
    this->get_parameter_or("bitrate", bitrate_, 800000);
    if (!this->get_parameter("subscribe_topics", sub_topics_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get param 'subscribe_topics'");
      throw std::runtime_error("Failed to get param 'subscribe_topics'");
    }

    if (!this->get_parameter("video_resolutions", resolutions_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get param 'video_resolutions'");
      throw std::runtime_error("Failed to get param 'video_resolutions'");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[ros2 constructor] output_fps: %d, bitrate: %d, topics: %s, resolutions: %s",
      output_fps_, bitrate_, format_topics(sub_topics_).c_str(),
      format_topics(resolutions_).c_str());

    auto topic_infos = this->get_topic_names_and_types();
    RCLCPP_INFO(this->get_logger(), "topic count: %zu", topic_infos.size());

    for (size_t i = 0; i < sub_topics_.size(); ++i) {
      const std::string & topic = sub_topics_[i];
      const std::string & resolution = resolutions_[i];
      int width, height;
      if (!get_image_size(resolution, width, height)) {
        RCLCPP_ERROR(this->get_logger(), "resolution format error: '%s'", resolution.c_str());
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "create encoder and publisher for topic '%s'", topic.c_str());

      // check topic type
      std::string topic_type;
      bool found_topic = false;
      for (const auto & topic_info : topic_infos) {
        if (topic_info.first == topic && !topic_info.second.empty()) {
          topic_type = topic_info.second[0];
          found_topic = true;
          break;
        }
      }

      // create publisher and encoder
      auto pub_topic = topic + "/h264";
      auto pub = this->create_publisher<foxglove_msgs::msg::CompressedVideo>(pub_topic, 10);
      publisher_map_.emplace(pub_topic, pub);

      try {
        encoder_map_.emplace(
          std::piecewise_construct,
          std::forward_as_tuple(pub_topic),
          std::forward_as_tuple(width, height, bitrate_, output_fps_));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "create encoder failed: %s", e.what());
        continue;
      }

      if (timer_map_.find(pub_topic) == timer_map_.end()) {
        auto timer = this->create_wall_timer(
          std::chrono::milliseconds(
            static_cast<int>(1.0 / static_cast<double>(output_fps_) * 1000)),
          [this, pub_topic]()
          {
            if (encoding_enabled_ && encoder_map_.find(pub_topic) != encoder_map_.end() &&
            publisher_map_.find(pub_topic) != publisher_map_.end())
            {
              try {
                auto frame = encoder_map_[pub_topic].encode_frame();
                if (frame) {
                  publisher_map_[pub_topic]->publish(*frame);
                }
              } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "encode frame failed: %s", e.what());
              }
            }
          });
        timer_map_.emplace(pub_topic, timer);
      }

      if (!found_topic || topic_type == "sensor_msgs/msg/Image") {
        RCLCPP_INFO(this->get_logger(), "create Image subscriber for topic '%s'", topic.c_str());
        auto img_sub = this->create_subscription<sensor_msgs::msg::Image>(
          topic, 10,
          [this, pub_topic](sensor_msgs::msg::Image::SharedPtr msg)
          {
            try {
              if (encoding_enabled_) {
                cv::Mat img = convert_to_cv_mat(*msg);
                if (!img.empty() && encoder_map_.find(pub_topic) != encoder_map_.end()) {
                  cv::Mat yuv_img;
                  cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);
                  encoder_map_[pub_topic].send_frame(yuv_img);
                }
              }
            } catch (const std::exception & e) {
              RCLCPP_ERROR(this->get_logger(), "process image failed: %s", e.what());
            }
          });
        image_sub_.emplace_back(img_sub);
      } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
        RCLCPP_INFO(
          this->get_logger(), "create CompressedImage subscriber for topic '%s'",
          topic.c_str());
        auto comp_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
          topic, 10,
          [this, pub_topic](sensor_msgs::msg::CompressedImage::SharedPtr msg)
          {
            try {
              if (encoding_enabled_) {
                cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
                if (!img.empty() && encoder_map_.find(pub_topic) != encoder_map_.end()) {
                  cv::Mat yuv_img;
                  cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);
                  encoder_map_[pub_topic].send_frame(yuv_img);
                }
              }
            } catch (const std::exception & e) {
              RCLCPP_ERROR(this->get_logger(), "process compressed image failed: %s", e.what());
            }
          });
        comp_image_sub_.emplace_back(comp_sub);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "topic '%s' type '%s' not supported", topic.c_str(),
          topic_type.c_str());
      }
    }

    // TODO(fei): need a service to control encode
    encoder_ctrl_ = this->create_service<std_srvs::srv::SetBool>(
      "/encoder_ctrl",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "encoder_ctrl was called");
        encoding_enabled_ = request->data;
        response->success = true;
        if (encoding_enabled_) {
          response->message = "encoder enabled";
          RCLCPP_INFO(this->get_logger(), "encoder enabled");
        } else {
          response->message = "encoder disabled";
          RCLCPP_INFO(this->get_logger(), "encoder disabled");
        }
        return;
      });

    RCLCPP_INFO(this->get_logger(), "create encoder control service: /encoder_ctrl");
  }

  ~CoEncoder() override
  {}

private:
  std::atomic<bool> encoding_enabled_{false};
  std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>>> image_sub_;
  std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>>>
  comp_image_sub_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>>>
  publisher_map_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr encoder_ctrl_;
  std::map<std::string, H264Encoder> encoder_map_;

  int output_fps_ = 30, bitrate_ = 800000;

  std::vector<std::string> sub_topics_;
  std::vector<std::string> resolutions_;

  std::map<std::string, rclcpp::TimerBase::SharedPtr> timer_map_;

  cv::Mat convert_to_cv_mat(const sensor_msgs::msg::Image & msg)
  {
    int width = msg.width;
    int height = msg.height;
    std::string encoding = msg.encoding;
    const uint8_t * data = msg.data.data();

    int cv_type = -1;

    if (encoding == "bgr8" || encoding == "rgb8") {
      cv_type = CV_8UC3;
    } else if (encoding == "mono8") {
      cv_type = CV_8UC1;
    } else if (encoding == "16UC1") {
      cv_type = CV_16UC1;
    } else if (encoding == "bgra8" || encoding == "rgba8") {
      cv_type = CV_8UC4;
    } else {
      throw std::runtime_error("Unsupported encoding type: " + encoding);
    }

    cv::Mat mat(height, width, cv_type, const_cast<uint8_t *>(data), msg.step);
    return mat;
  }

  static std::string format_topics(const std::vector<std::string> & topics)
  {
    std::string result;
    for (const auto & topic : topics) {
      result += "'" + topic + "' ";
    }
    return result;
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

  bool get_image_size(const std::string & resolution, int & width, int & height)
  {
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
      RCLCPP_ERROR(this->get_logger(), "invalid_argument: %s", e.what());
      return false;
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(), "out_of_range: %s", e.what());
      return false;
    }
  }
};

int main(int argc, char ** argv)
{
  RCLCPP_INFO(rclcpp::get_logger("MAIN"), "Init");
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CoEncoder>();
  RCLCPP_INFO(rclcpp::get_logger("MAIN"), "CoEncoder SPIN!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
