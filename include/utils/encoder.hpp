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


#ifndef UTILS__ENCODER_HPP_
#define UTILS__ENCODER_HPP_

#include <memory>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <utils/logger.hpp>

#ifdef ROS_VERSION_1
#include <ros/ros.h>
#include <coscene_msgs/CompressedVideo.h>
using CompressedVideo = coscene_msgs::CompressedVideo;
#else
#include <rclcpp/rclcpp.hpp>
// #include <coscene_msgs/msg/compressed_video.h>
#include <foxglove_msgs/msg/compressed_video.h>
// using CompressedVideo = coscene_msgs::msg::CompressedVideo;
using CompressedVideo = foxglove_msgs::msg::CompressedVideo;
#endif

using CompressedVideoPtr = std::shared_ptr<CompressedVideo>;

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class H264Encoder
{
public:
  H264Encoder()
  {
    H264Encoder(640, 480, 1600000, "");
  }

  H264Encoder(
    const int width, const int height, const int bitrate = 1600000,
    const std::string & topic = "")
  {
#ifdef ROS_VERSION_1
    avcodec_register_all();
#endif

    bitrate_ = bitrate;
    encoder_topic_ = topic;

    const char * encoder_names[] = {
      // "h264_nvenc",    // NVIDIA NVENC
      // "h264_qsv",      // Intel Quick Sync
      // "h264_amf",      // AMD VCE
      // "h264_vaapi",    // VAAPI (Linux hardware acceleration)
      "libx264"        // Software fallback
    };

    codec_ = nullptr;
    const char * selected_encoder = nullptr;

    for (const char * encoder_name : encoder_names) {
      codec_ = avcodec_find_encoder_by_name(encoder_name);
      if (codec_) {
        COLOG_INFO("create encoder with [%s] for topic [%s]", encoder_name, topic.c_str());
        selected_encoder = encoder_name;
        encoder_name_ = encoder_name;
        break;
      }
    }

    if (!codec_) {
      COLOG_INFO("No H.264 encoder found");
      throw std::runtime_error("No H.264 encoder found");
    }

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
      COLOG_ERROR("Could not allocate video codec context");
      throw std::runtime_error("Could not allocate video codec context");
    }

    codec_context_->width = width;
    codec_context_->height = height;

    codec_context_->bit_rate = bitrate_;
    codec_context_->rc_max_rate = bitrate_;
    codec_context_->rc_min_rate = bitrate_;
    codec_context_->rc_buffer_size = bitrate_;

    codec_context_->time_base = (AVRational) {1, 30};
    codec_context_->framerate = (AVRational) {30, 1};
    codec_context_->gop_size = 30;
    codec_context_->max_b_frames = 0;

    if (strcmp(selected_encoder, "h264_nvenc") == 0) {
      codec_context_->pix_fmt = AV_PIX_FMT_NV12;
    } else if (strcmp(selected_encoder, "h264_qsv") == 0) {
      codec_context_->pix_fmt = AV_PIX_FMT_NV12;
    } else if (strcmp(selected_encoder, "h264_vaapi") == 0) {
      codec_context_->pix_fmt = AV_PIX_FMT_VAAPI;
    } else {
      codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    }

    AVDictionary * codecOpts = nullptr;

    if (strcmp(selected_encoder, "h264_nvenc") == 0) {
      av_dict_set(&codecOpts, "preset", "llhq", 0);  // Low latency high quality
      av_dict_set(&codecOpts, "tune", "ll", 0);      // Low latency
      av_dict_set(&codecOpts, "rc", "cbr", 0);       // Constant bitrate
      av_dict_set(&codecOpts, "profile", "baseline", 0);
    } else if (strcmp(selected_encoder, "h264_qsv") == 0) {
      av_dict_set(&codecOpts, "preset", "veryfast", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "async_depth", "1", 0);
      av_dict_set(&codecOpts, "look_ahead", "0", 0);
      av_dict_set(&codecOpts, "ratecontrol", "cbr", 0);
    } else if (strcmp(selected_encoder, "h264_amf") == 0) {
      av_dict_set(&codecOpts, "quality", "speed", 0);
      av_dict_set(&codecOpts, "rc", "cbr", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
    } else if (strcmp(selected_encoder, "h264_vaapi") == 0) {
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "rc_mode", "CBR", 0);
    } else {
      av_dict_set(&codecOpts, "tune", "zerolatency", 0);
      av_dict_set(&codecOpts, "preset", "ultrafast", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "level", "4", 0);
      av_dict_set(&codecOpts, "refs", "1", 0);
      av_dict_set(&codecOpts, "me_method", "dia", 0);
      av_dict_set(&codecOpts, "subq", "1", 0);
      av_dict_set(&codecOpts, "trellis", "0", 0);
      av_dict_set(&codecOpts, "aq-mode", "0", 0);
      av_dict_set(&codecOpts, "me_range", "8", 0);
      av_dict_set(&codecOpts, "weightb", "0", 0);
      av_dict_set(&codecOpts, "8x8dct", "0", 0);
      av_dict_set(&codecOpts, "fast-pskip", "1", 0);
    }

    if (avcodec_open2(codec_context_, codec_, &codecOpts) < 0) {
      av_dict_free(&codecOpts);
      throw std::runtime_error("Could not open codec");
    }

    av_dict_free(&codecOpts);

    frame_ = av_frame_alloc();
    if (!frame_) {
      throw std::runtime_error("Could not allocate video frame");
    }

    frame_->format = codec_context_->pix_fmt;
    frame_->width = codec_context_->width;
    frame_->height = codec_context_->height;

    av_image_alloc(
      frame_->data, frame_->linesize, codec_context_->width,
      codec_context_->height, codec_context_->pix_fmt, 32);

    start_time_ = std::chrono::high_resolution_clock::time_point();
    initialized_ = false;

    COLOG_INFO(
      "H264 encoder initialized with bitrate: %d bps (%.2f Mbps)",
      bitrate_, bitrate_ / 1000000.0);
  }

  ~H264Encoder()
  {
    COLOG_INFO("destructor for topic [%s]", encoder_topic_.c_str());
    if (frame_) {
      av_freep(&frame_->data[0]);
      av_frame_free(&frame_);
    }
    if (codec_context_) {
      avcodec_free_context(&codec_context_);
    }
  }

  void send_frame(const cv::Mat & img)
  {
    received_ = true;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
      start_time_ = std::chrono::high_resolution_clock::now();
      initialized_ = true;
    }

    cv::Mat yuv_img;

    if (img.channels() == 1) {
      cv::Mat bgr_img;
      cv::cvtColor(img, bgr_img, cv::COLOR_GRAY2BGR);
      cv::cvtColor(bgr_img, yuv_img, cv::COLOR_BGR2YUV_I420);
    } else if (img.channels() == 3) {
      cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);
    } else if (img.channels() == 4) {
      cv::Mat bgr_img;
      cv::cvtColor(img, bgr_img, cv::COLOR_BGRA2BGR);
      cv::cvtColor(bgr_img, yuv_img, cv::COLOR_BGR2YUV_I420);
    } else {
      COLOG_ERROR("Unsupported image channels: %d", img.channels());
      return;
    }

    if (codec_context_->pix_fmt == AV_PIX_FMT_NV12) {
      int y_size = codec_context_->width * codec_context_->height;
      int uv_size = (codec_context_->width / 2) * (codec_context_->height / 2);

      memcpy(frame_->data[0], yuv_img.data, y_size);

      const uint8_t * u_src = yuv_img.data + y_size;
      const uint8_t * v_src = yuv_img.data + y_size + uv_size;
      uint8_t * uv_dst = frame_->data[1];

      for (int i = 0; i < uv_size; i++) {
        uv_dst[i * 2] = u_src[i];
        uv_dst[i * 2 + 1] = v_src[i];
      }
    } else {
      int y_size = codec_context_->width * codec_context_->height;
      int uv_size = (codec_context_->width / 2) * (codec_context_->height / 2);

      memcpy(frame_->data[0], yuv_img.data, y_size);
      memcpy(frame_->data[1], yuv_img.data + y_size, uv_size);
      memcpy(frame_->data[2], yuv_img.data + y_size + uv_size, uv_size);
    }
  }

  CompressedVideoPtr encode_frame()
  {
    if (!received_ || !initialized_) {
      return nullptr;
    }
    AVPacket pkt = {0};
    av_new_packet(&pkt, 0);

    std::lock_guard<std::mutex> lock(mutex_);

    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = current_time - start_time_;
    auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);

    // 1ms = 1000 ticks per second, so use milliseconds directly
    frame_->pts = elapsed_milliseconds.count();

    int ret = avcodec_send_frame(codec_context_, frame_);
    if (ret < 0) {
      return nullptr;
    }

    ret = avcodec_receive_packet(codec_context_, &pkt);
    if (ret == 0) {
      auto video_msg = CompressedVideo();
      video_msg.frame_id = "camera_frame";
      video_msg.data.assign(pkt.data, pkt.data + pkt.size);
      video_msg.format = "h264";

#ifdef ROS_VERSION_1
      video_msg.timestamp = ros::Time::now();
#else
      rclcpp::Clock clock(RCL_SYSTEM_TIME);
      video_msg.timestamp = clock.now();
#endif

      av_packet_unref(&pkt);
      return std::make_shared<CompressedVideo>(video_msg);
    }
    return nullptr;
  }

private:
  const AVCodec * codec_;
  AVFrame * frame_ = nullptr;
  AVCodecContext * codec_context_ = nullptr;
  std::string encoder_name_;
  std::string encoder_topic_;
  int bitrate_;

  std::mutex mutex_;

  std::chrono::high_resolution_clock::time_point start_time_;
  bool initialized_ = false;

  std::atomic<bool> received_{false};
};

#endif  // UTILS__ENCODER_HPP_
