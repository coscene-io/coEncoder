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
// #include <coscene_msgs/CompressedVideo.h>
// using CompressedVideo = coscene_msgs::CompressedVideo;
#include <foxglove_msgs/CompressedVideo.h>
using CompressedVideo = foxglove_msgs::CompressedVideo;
#else
#include <rclcpp/rclcpp.hpp>
// #include <coscene_msgs/msg/compressed_video.h>
// using CompressedVideo = coscene_msgs::msg::CompressedVideo;
#include <foxglove_msgs/msg/compressed_video.h>
using CompressedVideo = foxglove_msgs::msg::CompressedVideo;
#endif

using CompressedVideoPtr = std::shared_ptr<CompressedVideo>;

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

class H264Encoder
{
public:
  H264Encoder(
    const int width, const int height,
    const int bitrate = 1600000, const std::string & encoder_name = "libx264")
  {
#ifdef ROS_VERSION_1
    avcodec_register_all();
#endif

    bitrate_ = bitrate;
    encoder_name_ = encoder_name;
    codec_ = avcodec_find_encoder_by_name(encoder_name_.c_str());
    if (!codec_) {
      COLOG_INFO("create encoder with [%s] failed, not found", encoder_name.c_str());
      throw std::runtime_error("encoder not found");
    }
    COLOG_INFO("create encoder with [%s]", encoder_name.c_str());

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
      COLOG_ERROR("Could not allocate video codec context");
      throw std::runtime_error("Could not allocate video codec context");
    }

    codec_context_->width = width;
    codec_context_->height = height;

    // codec_context_->bit_rate = bitrate_;
    // codec_context_->rc_max_rate = bitrate_;
    // codec_context_->rc_min_rate = bitrate_;
    // codec_context_->rc_buffer_size = bitrate_;
    // codec_context_->rc_initial_buffer_occupancy = bitrate_ / 2;  // Initial buffer occupancy 

    codec_context_->time_base = (AVRational) {1, 1000};
    codec_context_->gop_size = 30;
    codec_context_->max_b_frames = 0;

    if (encoder_name_ == "h264_nvenc") {
      codec_context_->pix_fmt = AV_PIX_FMT_NV12;
    } else if (encoder_name_ == "h264_qsv") {
      codec_context_->pix_fmt = AV_PIX_FMT_NV12;
    } else if (encoder_name_ == "h264_vaapi") {
      codec_context_->pix_fmt = AV_PIX_FMT_VAAPI;
    } else {
      codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    }

    AVDictionary * codecOpts = nullptr;

    if (encoder_name_ == "h264_nvenc") {
      av_dict_set(&codecOpts, "preset", "llhq", 0);  // Low latency high quality
      av_dict_set(&codecOpts, "tune", "ll", 0);      // Low latency
      av_dict_set(&codecOpts, "rc", "cbr", 0);       // Constant bitrate
      av_dict_set(&codecOpts, "profile", "baseline", 0);
    } else if (encoder_name_ == "h264_qsv") {
      av_dict_set(&codecOpts, "preset", "veryfast", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "async_depth", "1", 0);
      av_dict_set(&codecOpts, "look_ahead", "0", 0);
      av_dict_set(&codecOpts, "ratecontrol", "cbr", 0);
    } else if (encoder_name_ == "h264_amf") {
      av_dict_set(&codecOpts, "quality", "speed", 0);
      av_dict_set(&codecOpts, "rc", "cbr", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
    } else if (encoder_name_ == "h264_vaapi") {
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "rc_mode", "CBR", 0);
    } else {
      av_dict_set(&codecOpts, "preset", "ultrafast", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "rc", "cbr", 0);  // Constant bitrate
      av_dict_set(&codecOpts, "bitrate", std::to_string(bitrate_).c_str(), 0);  // Explicit bitrate
      av_dict_set(&codecOpts, "maxrate", std::to_string(bitrate_).c_str(), 0);
      av_dict_set(&codecOpts, "minrate", std::to_string(bitrate_).c_str(), 0);
      av_dict_set(&codecOpts, "bufsize", std::to_string(bitrate_ / 4).c_str(), 0);
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

    COLOG_INFO(
      "H264 encoder initialized with bitrate: %d bps (%.2f Mbps)",
      bitrate_, bitrate_ / 1000000.0);
  }

  ~H264Encoder()
  {
    if (frame_) {
      av_freep(&frame_->data[0]);
      av_frame_free(&frame_);
    }
    if (codec_context_) {
      avcodec_free_context(&codec_context_);
    }
  }


  /**
   * @brief Send a frame to the encoder for processing
   *
   * This function converts the input image to the required format and sends it to the H.264 encoder.
   * The encoder will process the frame and store it in its internal buffer.
   *
   * @param img Input image in OpenCV Mat format (BGR, RGB, or grayscale)
   * @param timestamp Frame timestamp in MILISECONDS (used for PTS calculation)
   *
   */
  void send_frame(const cv::Mat & img, const int64_t & timestamp)
  {
    try {
      received_ = true;
      std::lock_guard<std::mutex> lock(mutex_);

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
        const int y_size = codec_context_->width * codec_context_->height;
        const int uv_size = (codec_context_->width / 2) * (codec_context_->height / 2);

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

      frame_->pts = timestamp;

      const int ret = avcodec_send_frame(codec_context_, frame_);
      if (ret < 0) {
        char err_buf[128];
        av_strerror(ret, err_buf, sizeof(err_buf));
        COLOG_WARN("send frame to encoder failed: %s", err_buf);
      }
    } catch (const std::exception & e) {
      COLOG_ERROR("Exception in send_frame: %s", e.what());
      throw;
    } catch (...) {
      COLOG_ERROR("Unknown exception in send_frame");
      throw;
    }
  }

  /**
   * @brief Retrieve an encoded frame from the encoder
   *
   * This function attempts to get an encoded H.264 frame from the encoder's output buffer.
   * The encoder processes frames asynchronously, so this function may not always return a frame.
   *
   * @return std::shared_ptr<CompressedVideo> Encoded video frame, or nullptr if no frame is available
   */
  CompressedVideoPtr encode_frame()
  {
    if (!received_) {
      return nullptr;
    }
    AVPacket pkt = {nullptr};
    av_new_packet(&pkt, 0);

    std::lock_guard<std::mutex> lock(mutex_);

    const auto ret = avcodec_receive_packet(codec_context_, &pkt);
    if (ret == 0) {
      auto video_msg = CompressedVideo();
      video_msg.frame_id = "camera_frame";
      video_msg.data.assign(pkt.data, pkt.data + pkt.size);
      video_msg.format = "h264";

#ifdef ROS_VERSION_1
      video_msg.timestamp = ros::Time(pkt.pts / 1e3);
#else
      video_msg.timestamp = rclcpp::Time(pkt.pts * 1000000);
#endif

      av_packet_unref(&pkt);
      return std::make_shared<CompressedVideo>(video_msg);
    } else {
      // Always unref the packet, even on failure, to prevent memory leak
      av_packet_unref(&pkt);
    }
    return nullptr;
  }

private:
  const AVCodec * codec_;
  AVFrame * frame_ = nullptr;
  AVCodecContext * codec_context_ = nullptr;
  std::string encoder_name_;
  int bitrate_;

  std::mutex mutex_;
  std::atomic<bool> received_{false};
};

#endif  // UTILS__ENCODER_HPP_
