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
  H264Encoder(const int width, const int height, const TopicParam & param)
  {
#ifdef ROS_VERSION_1
    avcodec_register_all();
#endif

    bitrate_ = param.bitrate;
    encoder_topic_ = param.output_topic;
    encoder_name_ = param.encoder_name;
    codec_ = avcodec_find_encoder_by_name(encoder_name_.c_str());
    if (!codec_) {
      COLOG_INFO(
        "create [%s] encoder with [%s] failed, not found",
        encoder_topic_.c_str(), encoder_name_.c_str());
      throw std::runtime_error("encoder not found");
    }
    COLOG_INFO(
      "create [%s] encoder with [%s], width: %d, height: %d",
      encoder_topic_.c_str(), encoder_name_.c_str(), width, height);

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
      COLOG_ERROR("Could not allocate video codec context for topic [%s]", encoder_topic_.c_str());
      throw std::runtime_error("Could not allocate video codec context");
    }

    codec_context_->width = width;
    codec_context_->height = height;
    codec_context_->bit_rate = bitrate_;
    codec_context_->gop_size = 30;
    codec_context_->max_b_frames = 0;
    codec_context_->time_base = (AVRational) {1, 1000};
    codec_context_->framerate = (AVRational) {30, 1};

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
      av_dict_set(&codecOpts, "preset", param.encode_preset.c_str(), 0);
      av_dict_set(&codecOpts, "tune", param.encode_tune.c_str(), 0);
      av_dict_set(&codecOpts, "rc", "vbr", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "bitrate", std::to_string(bitrate_).c_str(), 0);
      av_dict_set(&codecOpts, "maxrate", std::to_string(bitrate_ * 1.5).c_str(), 0);
    } else if (encoder_name_ == "h264_qsv") {
      av_dict_set(&codecOpts, "preset", param.encode_preset.c_str(), 0);
      av_dict_set(&codecOpts, "tune", param.encode_tune.c_str(), 0);
      av_dict_set(&codecOpts, "async_depth", "1", 0);
      av_dict_set(&codecOpts, "look_ahead", "0", 0);
      av_dict_set(&codecOpts, "ratecontrol", "vbr", 0);
      av_dict_set(&codecOpts, "bitrate", std::to_string(bitrate_).c_str(), 0);
      av_dict_set(&codecOpts, "maxrate", std::to_string(bitrate_ * 1.5).c_str(), 0);
    } else if (encoder_name_ == "h264_amf") {
      av_dict_set(&codecOpts, "preset", param.encode_preset.c_str(), 0);
      av_dict_set(&codecOpts, "tune", param.encode_tune.c_str(), 0);
      av_dict_set(&codecOpts, "quality", "speed", 0);
      av_dict_set(&codecOpts, "rc", "vbr", 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "bitrate", std::to_string(bitrate_).c_str(), 0);
      av_dict_set(&codecOpts, "maxrate", std::to_string(bitrate_ * 1.5).c_str(), 0);
    } else if (encoder_name_ == "h264_vaapi") {
      av_dict_set(&codecOpts, "preset", param.encode_preset.c_str(), 0);
      av_dict_set(&codecOpts, "tune", param.encode_tune.c_str(), 0);
      av_dict_set(&codecOpts, "rc_mode", "VBR", 0);
      av_dict_set(&codecOpts, "bitrate", std::to_string(bitrate_).c_str(), 0);
      av_dict_set(&codecOpts, "maxrate", std::to_string(bitrate_ * 1.5).c_str(), 0);
    } else {
      av_dict_set(&codecOpts, "preset", param.encode_preset.c_str(), 0);
      av_dict_set(&codecOpts, "tune", param.encode_tune.c_str(), 0);
      av_dict_set(&codecOpts, "profile", "baseline", 0);
      av_dict_set(&codecOpts, "level", "3.1", 0);

      // Thread optimization for multiple encoders
      av_dict_set(&codecOpts, "threads", "1", 0);          // Limit threads per encoder
      av_dict_set(&codecOpts, "sliced-threads", "1", 0);   // Use sliced threading

      // // Frame rate guarantee settings (minimal encoding complexity)
      // av_dict_set(&codecOpts, "refs", "1", 0);             // Single reference frame
      // av_dict_set(&codecOpts, "me_method", "dia", 0);      // Diamond search (fastest)
      // av_dict_set(&codecOpts, "subq", "1", 0);              // Minimal subpixel refinement
      // av_dict_set(&codecOpts, "trellis", "0", 0);           // Disable trellis quantization
      // av_dict_set(&codecOpts, "aq-mode", "0", 0);           // Disable adaptive quantization
      // av_dict_set(&codecOpts, "me_range", "4", 0);          // Small motion estimation range
      // av_dict_set(&codecOpts, "weightb", "0", 0);           // Disable weighted B-frames
      // av_dict_set(&codecOpts, "8x8dct", "0", 0);            // Disable 8x8 DCT
      // av_dict_set(&codecOpts, "fast-pskip", "1", 0);        // Enable fast P-skip
      //
      // // Additional settings for frame rate guarantee
      // av_dict_set(&codecOpts, "rc-lookahead", "0", 0);      // No lookahead for immediate encode
      // av_dict_set(&codecOpts, "no-scenecut", "1", 0);       // Disable scene cut detection
      // av_dict_set(&codecOpts, "bframes", "0", 0);           // No B-frames for simplicity
      // av_dict_set(&codecOpts, "b-adapt", "0", 0);           // Disable B-frame adaptation
      // av_dict_set(&codecOpts, "direct", "none", 0);         // Disable direct mode
      // av_dict_set(&codecOpts, "no-cabac", "1", 0);          // Disable CABAC for lower CPU
      // av_dict_set(&codecOpts, "no-deblock", "1", 0);        // Disable deblocking filter
      //
      // // Minimize internal buffering
      // av_dict_set(&codecOpts, "sync-lookahead", "0", 0);    // Disable sync lookahead
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
      "[%s] encoder initialized with bitrate: %d bps (%.2f Mbps)", encoder_topic_.c_str(),
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
   * @return int 0 on success, AVERROR(EAGAIN) if buffer is full, negative on other errors
   *
   */
  int send_frame(const cv::Mat & img, const int64_t & timestamp)
  {
    try {
      if (!received_) {
        received_ = true;
        first_frame_timestamp = timestamp;
      }
      last_frame_timestamp = timestamp;
      std::lock_guard<std::mutex> lock(mutex_);

      cv::Mat yuv_img;

      if (img.channels() == 1) {
        cv::Mat bgr_img;
        cv::cvtColor(img, bgr_img, cv::COLOR_GRAY2BGR);
        cv::cvtColor(bgr_img, yuv_img, cv::COLOR_BGR2YUV_I420);
      } else if (img.channels() == 3) {
        cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV_I420);
      } else if (img.channels() == 4) {
        cv::cvtColor(img, yuv_img, cv::COLOR_BGRA2YUV_I420);
      } else {
        COLOG_ERROR("Unsupported image channels: %d", img.channels());
        return AVERROR(EINVAL);
      }

      if (codec_context_->pix_fmt == AV_PIX_FMT_NV12) {
        const uint8_t * y_src = yuv_img.data;
        for (int i = 0; i < codec_context_->height; i++) {
          memcpy(
            frame_->data[0] + i * frame_->linesize[0],
            y_src + i * codec_context_->width,
            codec_context_->width);
        }

        const uint8_t * u_src = yuv_img.data + codec_context_->width * codec_context_->height;
        const uint8_t * v_src = u_src + (codec_context_->width / 2) * (codec_context_->height / 2);

        for (int i = 0; i < codec_context_->height / 2; i++) {
          uint8_t * uv_dst = frame_->data[1] + i * frame_->linesize[1];
          for (int j = 0; j < codec_context_->width / 2; j++) {
            uv_dst[j * 2] = u_src[i * (codec_context_->width / 2) + j];
            uv_dst[j * 2 + 1] = v_src[i * (codec_context_->width / 2) + j];
          }
        }
      } else {
        const uint8_t * y_src = yuv_img.data;
        for (int i = 0; i < codec_context_->height; i++) {
          memcpy(
            frame_->data[0] + i * frame_->linesize[0],
            y_src + i * codec_context_->width,
            codec_context_->width);
        }

        const uint8_t * u_src = yuv_img.data + codec_context_->width * codec_context_->height;
        for (int i = 0; i < codec_context_->height / 2; i++) {
          memcpy(
            frame_->data[1] + i * frame_->linesize[1],
            u_src + i * (codec_context_->width / 2),
            codec_context_->width / 2);
        }

        const uint8_t * v_src = u_src + (codec_context_->width / 2) * (codec_context_->height / 2);
        for (int i = 0; i < codec_context_->height / 2; i++) {
          memcpy(
            frame_->data[2] + i * frame_->linesize[2],
            v_src + i * (codec_context_->width / 2),
            codec_context_->width / 2);
        }
      }

      frame_->pts = timestamp;

      const int ret = avcodec_send_frame(codec_context_, frame_);
      if (ret < 0) {
        if (ret == AVERROR(EAGAIN)) {
          ++send_eagain_count_;
          COLOG_DEBUG(
            "[%s] Encoder buffer full (EAGAIN), frame may be delayed",
            encoder_topic_.c_str());
          return ret;
        } else {
          char err_buf[128];
          av_strerror(ret, err_buf, sizeof(err_buf));
          COLOG_WARN(
            "send frame to [%s] encoder failed: %s (error code: %d)",
            encoder_topic_.c_str(), err_buf, ret);
          return ret;
        }
      }
      ++frames_sent_;
      return 0;
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
      ++packets_received_;
      return std::make_shared<CompressedVideo>(video_msg);
    } else {
      if (ret == AVERROR(EAGAIN)) {
        ++recv_eagain_count_;
        COLOG_DEBUG(
          "[%s] encode_frame: EAGAIN - encoder needs more input before producing output",
          encoder_topic_.c_str());
      } else if (ret != AVERROR_EOF) {
        char err_buf[128];
        av_strerror(ret, err_buf, sizeof(err_buf));
        COLOG_WARN(
          "[%s] encode_frame failed: %s (error code: %d)",
          encoder_topic_.c_str(), err_buf, ret);
      }
      av_packet_unref(&pkt);
    }
    return nullptr;
  }

  void print_stats() const
  {
    const auto last_frame_ts = last_frame_timestamp.load();
    const auto first_frame_ts = first_frame_timestamp.load();
    COLOG_DEBUG(
      "📊  [%s] Sent: %lu, Output: %lu, Send_EAGAIN: %lu, Recv_EAGAIN: %lu, output FPS: %.2f, "
      "last_frame_timestamp: %ld, first_frame_timestamp: %ld",
      encoder_topic_.c_str(),
      frames_sent_.load(),
      packets_received_.load(),
      send_eagain_count_.load(),
      recv_eagain_count_.load(),
      frames_sent_ >
      0 ? (packets_received_.load() * 1000.0 / (last_frame_ts - first_frame_ts)) : 0.0,
      last_frame_ts,
      first_frame_ts
    );
  }

private:
  const AVCodec * codec_;
  AVFrame * frame_ = nullptr;
  AVCodecContext * codec_context_ = nullptr;
  std::string encoder_name_;
  std::string encoder_topic_;
  int bitrate_;

  std::mutex mutex_;
  std::atomic<bool> received_{false};

  std::atomic<int64_t> first_frame_timestamp{0};
  std::atomic<int64_t> last_frame_timestamp{0};

  // Performance statistics
  std::atomic<uint64_t> frames_sent_{0};
  std::atomic<uint64_t> packets_received_{0};
  std::atomic<uint64_t> send_eagain_count_{0};
  std::atomic<uint64_t> recv_eagain_count_{0};
};

#endif  // UTILS__ENCODER_HPP_
