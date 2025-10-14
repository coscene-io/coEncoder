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

#ifndef UTILS__ENCODER_WORKER_HPP_
#define UTILS__ENCODER_WORKER_HPP_

#include <atomic>
#include <chrono>
#include <thread>
#include <functional>
#include <deque>
#include <opencv2/opencv.hpp>
#include <utility>
#include <string>
#include "concurrentqueue.hpp"
#include "utils/encoder.hpp"
#include "utils/logger.hpp"

// Frame data for encoding queue
struct FrameData
{
  cv::Mat image;
  int64_t timestamp;

  FrameData() = default;
  FrameData(const cv::Mat & img, int64_t ts)
  : image(img), timestamp(ts) {}
};

// Frame rate control info
struct FrameRateInfo
{
  int32_t output_framerate;
  std::deque<int64_t> timestamp_window;
  static constexpr int64_t WINDOW_SIZE_MS = 2000;

  explicit FrameRateInfo(const int32_t & framerate)
  : output_framerate(framerate) {}
};

class EncoderWorker
{
public:
  using PublishCallback = std::function<void (CompressedVideoPtr)>;

  EncoderWorker(const int & width, const int & height, const TopicParam & param, PublishCallback cb)
  : encoder_(width, height, param),
    frame_rate_info_(param.output_frame_rate),
    publish_callback_(std::move(cb)),
    topic_(param.output_topic)
  {
    // Start worker thread
    worker_thread_ = std::thread(&EncoderWorker::run, this);
  }

  ~EncoderWorker()
  {
    stop_ = true;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  // Non-copyable
  EncoderWorker(const EncoderWorker &) = delete;
  EncoderWorker & operator=(const EncoderWorker &) = delete;

  // Enqueue a frame for encoding
  void enqueue_frame(const cv::Mat & image, int64_t timestamp)
  {
    frame_queue_.enqueue(FrameData(image, timestamp));
  }

  // Get encoder for statistics
  H264Encoder & get_encoder()
  {
    return encoder_;
  }

  // Get approximate queue size
  size_t get_queue_size() const
  {
    return frame_queue_.size_approx();
  }

private:
  void run()
  {
    FrameData frame;
    while (!stop_) {
      if (frame_queue_.try_dequeue(frame)) {
        try {
          if (!resample_fps(frame.timestamp)) {
            continue;
          }
          int send_ret = encoder_.send_frame(frame.image, frame.timestamp);
          if (send_ret == AVERROR(EAGAIN)) {
            int drained_count = 0;
            while (true) {
              auto encoded_frame = encoder_.encode_frame();
              if (encoded_frame) {
                publish_callback_(encoded_frame);
                drained_count++;
              } else {
                break;
              }
            }
            send_ret = encoder_.send_frame(frame.image, frame.timestamp);
          }
          auto encoded_frame = encoder_.encode_frame();
          if (encoded_frame) {
            publish_callback_(encoded_frame);
          }
        } catch (const std::exception & e) {
          COLOG_ERROR(
            "Encoding error for topic %s: %s",
            topic_.c_str(), e.what());
        }
      } else {
        // No frame available, sleep briefly to avoid busy-waiting
        // 1ms sleep is enough to reduce CPU usage while maintaining responsiveness
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  bool resample_fps(int64_t timestamp)
  {
    // If output frame rate is 0, pass all frames
    if (frame_rate_info_.output_framerate == 0) {
      return true;
    }

    const auto current_time_ms = timestamp;

    // Remove old timestamps
    while (!frame_rate_info_.timestamp_window.empty() &&
      current_time_ms - frame_rate_info_.timestamp_window.front() >
      FrameRateInfo::WINDOW_SIZE_MS)
    {
      frame_rate_info_.timestamp_window.pop_front();
    }

    // If window is empty, accept this frame
    if (frame_rate_info_.timestamp_window.empty()) {
      frame_rate_info_.timestamp_window.push_back(current_time_ms);
      return true;
    }

    // Calculate current frequency
    const int64_t time_span = current_time_ms - frame_rate_info_.timestamp_window.front();
    if (time_span <= 0) {
      // Timestamp not increasing or going backwards, skip this frame
      COLOG_WARN(
        "[%s] Timestamp not increasing: current=%ld, front=%ld",
        topic_.c_str(), current_time_ms,
        frame_rate_info_.timestamp_window.front());
      return false;
    }

    // Calculate frequency: (frames - 1) / time_span, because N frames have N-1 intervals
    const double current_freq =
      ((frame_rate_info_.timestamp_window.size()) * 1000.0) / time_span;

    // Debug: Log when skipping frames
    if (current_freq >= frame_rate_info_.output_framerate) {
      return false;
    }

    frame_rate_info_.timestamp_window.push_back(current_time_ms);
    return true;
  }

  moodycamel::ConcurrentQueue<FrameData> frame_queue_;
  std::thread worker_thread_;
  std::atomic<bool> stop_{false};
  H264Encoder encoder_;
  FrameRateInfo frame_rate_info_;
  PublishCallback publish_callback_;
  std::string topic_;
};

#endif  // UTILS__ENCODER_WORKER_HPP_
