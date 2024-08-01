//////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////

#include <thread>
#include <chrono>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <foxglove_msgs/CompressedVideo.h>
#include <opencv2/opencv.hpp>
#include "encoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#define TARGET_WIDTH 640

class CoEncoder {
public:
    CoEncoder() {
        ros::NodeHandle nh("~");

        nh.param("output_fps", output_fps_, 30);
        nh.param("bitrate", bitrate_, 800000);
        ROS_INFO("[constructor] output_fps: %d, bitrate: %d", output_fps_, bitrate_);

        if (!nh.getParam("topics", sub_topics_)) {
            ROS_ERROR("Failed to get param 'subscribe_topics'");
            ros::shutdown();
        }

        if (!nh.getParam("resolutions", resolutions_)) {
            ROS_ERROR("Failed to get param 'subscribe_topics'");
            ros::shutdown();
        }
        ROS_INFO("[constructor] sub_topics: %s", format_topics(sub_topics_).c_str());

        get_all_topics_and_type();
        ros::Duration interval(1.0 / static_cast<double>(output_fps_));

        for (size_t i = 0; i < sub_topics_.size(); ++i) {
            const std::string& topic = sub_topics_[i];
            const std::string& resolution = resolutions_[i];

            int width = 0, height = 0;
            if (!get_resized_image_size(resolution, width, height)) {
                ROS_WARN("Failed to parse resolution '%s' for topic: %s", resolution.c_str(), topic.c_str());
                continue;
            }
            ROS_INFO("resize %s image to : %d*%d", topic.c_str(), width, height);

            std::string pub_topic = topic + "/h264";
            encoder_map_.emplace(std::piecewise_construct,
                                 std::forward_as_tuple(pub_topic),
                                 std::forward_as_tuple(width, height, bitrate_, output_fps_));
            ros::Publisher pub = nh.advertise<foxglove_msgs::CompressedVideo>(pub_topic, 1);

            std::string msg_type = topic_map_[topic];
            if (msg_type != "sensor_msgs/CompressedImage" && msg_type != "sensor_msgs/Image") {
                ROS_WARN("Unsupported message type '%s' for topic '%s'", msg_type.c_str(), topic.c_str());
                continue;
            }

            ros::Subscriber sub;
            if (msg_type == "sensor_msgs/CompressedImage") {
                sub = nh.subscribe<sensor_msgs::CompressedImage>(
                        topic, 1,
                        [this, pub_topic, height](const sensor_msgs::CompressedImage::ConstPtr& msg) {
                            process_image(cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR), pub_topic, height);
                        });
            } else {
                sub = nh.subscribe<sensor_msgs::Image>(
                        topic, 1,
                        [this, pub_topic, height](const sensor_msgs::Image::ConstPtr& msg) {
                            process_image(convertToCvMat(*msg), pub_topic, height);
                        });
            }
            subscribers_.emplace_back(sub);
            publisher_map_.emplace(pub_topic, pub);
        }

        for (const auto& pub_topic : sub_topics_) {
            if (timer_map_.find(pub_topic) == timer_map_.end()) {
                auto timer = nh.createTimer(interval,
                                            [this, pub_topic](const ros::TimerEvent&) {
                                                auto frame = encoder_map_[pub_topic + "/h264"].encode_frame();
                                                if (frame) {
                                                    publisher_map_[pub_topic + "/h264"].publish(frame);
                                                }
                                            });
                timer_map_.emplace(pub_topic, timer);
            }
        }

    }

    ~CoEncoder() = default;

private:
    cv::Mat convertToCvMat(const sensor_msgs::Image& img_msg) {
        int cv_type = CV_8UC3;
        if (img_msg.encoding == sensor_msgs::image_encodings::BGR8) {
            cv_type = CV_8UC3;
        } else if (img_msg.encoding == sensor_msgs::image_encodings::MONO8) {
            cv_type = CV_8UC1;
        } else if (img_msg.encoding == sensor_msgs::image_encodings::RGB8) {
            cv_type = CV_8UC3;
        } else if (img_msg.encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
            cv_type = CV_8UC1;
        } else if (img_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_type = CV_16UC1;
        } else {
            ROS_ERROR("Unsupported encoding type: %s", img_msg.encoding.c_str());
            return cv::Mat(); // Return an empty Mat in case of unsupported encoding
        }

        cv::Mat image(img_msg.height, img_msg.width, cv_type, const_cast<uchar*>(img_msg.data.data()), img_msg.step);
        if (img_msg.encoding == sensor_msgs::image_encodings::RGB8) {
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        }
        return image;
    }


    void process_image(cv::Mat img, const std::string& pub_topic, int height) {
        if (img.empty()) {
            ROS_WARN("Empty image received");
            return;
        }

        if (img.cols != TARGET_WIDTH || img.rows != height) {
            cv::resize(img, img, cv::Size(TARGET_WIDTH, height));
        }

        cv::Mat yuv_img;
        cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);

        encoder_map_[pub_topic].send_frame(yuv_img);
    }

    std::string format_topics(const std::vector<std::string>& topics) const {
        std::string result;
        for (const auto& topic : topics) {
            result += "'" + topic + "' ";
        }
        return result;
    }

    void get_all_topics_and_type() {
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        for (const auto& topic : topics) {
            topic_map_.emplace(topic.name, topic.datatype);
        }
    }

    static bool get_resized_image_size(const std::string& resolution, int& width, int& height) {
        std::string trim_str = trim(resolution);
        size_t pos = trim_str.find('*');
        if (pos == std::string::npos) {
            ROS_WARN("Illegal resolution format: %s", trim_str.c_str());
            return false;
        }

        try {
            int src_width = std::stoi(trim_str.substr(0, pos));
            int src_height = std::stoi(trim_str.substr(pos + 1));
            double scale = static_cast<double>(TARGET_WIDTH) / src_width;

            width = TARGET_WIDTH;
            height = static_cast<int>(src_height * scale);
            return true;
        } catch (const std::invalid_argument& e) {
            ROS_WARN("invalid_argument: %s", e.what());
            return false;
        } catch (const std::out_of_range& e) {
            ROS_WARN("out_of_range: %s", e.what());
            return false;
        }
    }

    static std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(' ');
        if (std::string::npos == first) {
            return str;
        }
        size_t last = str.find_last_not_of(' ');
        return str.substr(first, (last - first + 1));
    }

    std::vector<std::string> sub_topics_;
    std::vector<std::string> resolutions_;

    std::vector<ros::Subscriber> subscribers_;
    std::map<std::string, ros::Publisher> publisher_map_;

    std::map<std::string, H264Encoder> encoder_map_;
    std::map<std::string, ros::Timer> timer_map_;

    std::map<std::string, std::string> topic_map_;

    int output_fps_ = 30, bitrate_ = 800000;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coencoder");
    CoEncoder node;
    ros::spin();
    return 0;
}
