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


#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include <memory>
#include <opencv2/opencv.hpp>

#ifdef ROS_VERSION_1
#include <ros/ros.h>
#include <foxglove_msgs/CompressedVideo.h>
using CompressedVideo = foxglove_msgs::CompressedVideo;
#else
#include <rclcpp/rclcpp.hpp>
#include <foxglove_msgs/msg/compressed_video.h>
using CompressedVideo = foxglove_msgs::msg::CompressedVideo;
#endif

using CompressedVideoPtr = std::shared_ptr<CompressedVideo>;

extern "C"
{
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
    H264Encoder(640, 480, 400000, 30);
  }

  H264Encoder(int width, int height, int bitrate, int fps)
  {
#ifdef ROS_VERSION_1
    avcodec_register_all();
#endif
    codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec_) {
      throw std::runtime_error("H.264 codec not found");
    }

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
      throw std::runtime_error("Could not allocate video codec context");
    }

    codec_context_->bit_rate = bitrate;
    codec_context_->width = width;
    codec_context_->height = height;
    codec_context_->time_base = {1, fps};
    codec_context_->framerate = (AVRational) {fps, 1};
    codec_context_->gop_size = 10;
    codec_context_->max_b_frames = 0;
    codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;

    AVDictionary * codecOpts = nullptr;
    av_dict_set(&codecOpts, "tune", "zerolatency", 0);
    av_dict_set(&codecOpts, "preset", "ultrafast", 0);

    if (avcodec_open2(codec_context_, codec_, &codecOpts) < 0) {
      throw std::runtime_error("Could not open codec");
    }

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
  }

  ~H264Encoder()
  {
    avcodec_free_context(&codec_context_);
    av_frame_free(&frame_);
    av_freep(&frame_->data[0]);
  }

  void send_frame(const cv::Mat & img)
  {
    received_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    int y_size = codec_context_->width * codec_context_->height;
    int uv_size = (codec_context_->width / 2) * (codec_context_->height / 2);

    memcpy(frame_->data[0], img.data, y_size);
    memcpy(frame_->data[1], img.data + y_size, uv_size);
    memcpy(frame_->data[2], img.data + y_size + uv_size, uv_size);
  }

  CompressedVideoPtr encode_frame()
  {
    if (!received_) {
      return nullptr;
    }
    AVPacket pkt = {0};
    av_new_packet(&pkt, 0);

    std::lock_guard<std::mutex> lock(mutex_);
    frame_->pts = pts_++;
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

  std::mutex mutex_;

  int64_t pts_ = 0;

  std::atomic<bool> received_{false};
};

#endif  // ENCODER_HPP_
