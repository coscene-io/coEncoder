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

#ifndef ROS1_WS_ENCODER_HPP
#define ROS1_WS_ENCODER_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <foxglove_msgs/CompressedVideo.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class H264Encoder {
public:
    H264Encoder() {
        H264Encoder(640, 480, 400000, 30);
    }

    H264Encoder(int width, int height, int bitrate, int fps) {
        codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec_) {
            ROS_ERROR("H.264 codec not found");
            ros::shutdown();
        }

        codec_context_ = avcodec_alloc_context3(codec_);
        if (!codec_context_) {
            ROS_ERROR("Could not allocate video codec context");
            ros::shutdown();
        }

        codec_context_->bit_rate = bitrate;
        codec_context_->width = width;
        codec_context_->height = height;
        codec_context_->time_base = {1, fps};
        codec_context_->framerate = (AVRational) {fps, 1};
        codec_context_->gop_size = 10;
        codec_context_->max_b_frames = 0;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;

        AVDictionary *codecOpts = nullptr;
        av_dict_set(&codecOpts, "tune", "zerolatency", 0);
        av_dict_set(&codecOpts, "preset", "ultrafast", 0);

        if (avcodec_open2(codec_context_, codec_, &codecOpts) < 0) {
            ROS_ERROR("Could not open codec");
            ros::shutdown();
        }

        frame_ = av_frame_alloc();
        if (!frame_) {
            ROS_ERROR("Could not allocate video frame");
            ros::shutdown();
        }

        frame_->format = codec_context_->pix_fmt;
        frame_->width = codec_context_->width;
        frame_->height = codec_context_->height;

        av_image_alloc(frame_->data, frame_->linesize, codec_context_->width,
                       codec_context_->height, codec_context_->pix_fmt, 32);

    }

    ~H264Encoder() {
        avcodec_free_context(&codec_context_);
        av_frame_free(&frame_);
        av_freep(&frame_->data[0]);
    }

    void send_frame(const cv::Mat &img) {
        received_ = true;
        std::lock_guard<std::mutex> lock(mutex_);
        int y_size = codec_context_->width * codec_context_->height;
        int uv_size = (codec_context_->width / 2) * (codec_context_->height / 2);

        memcpy(frame_->data[0], img.data, y_size);
        memcpy(frame_->data[1], img.data + y_size, uv_size);
        memcpy(frame_->data[2], img.data + y_size + uv_size, uv_size);
    }

    foxglove_msgs::CompressedVideoPtr encode_frame() {
        if (!received_) return nullptr;
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = nullptr;
        pkt.size = 0;

        std::lock_guard<std::mutex> lock(mutex_);
        frame_->pts = pts_++;
        int ret = avcodec_send_frame(codec_context_, frame_);
        if (ret < 0) {
            ROS_ERROR("Error sending frame for encoding");
            return nullptr;
        }

        ret = avcodec_receive_packet(codec_context_, &pkt);
        if (ret == 0) {
            auto video_msg = foxglove_msgs::CompressedVideo();
            video_msg.timestamp = ros::Time::now();
            video_msg.frame_id = "camera_frame";
            video_msg.data.assign(pkt.data, pkt.data + pkt.size);
            video_msg.format = "h264";

//            ROS_INFO("encoded size: %d, %02x %02x %02x %02x   %02x %02x %02x %02x   %02x %02x %02x %02x   %02x %02x %02x %02x", pkt.size,
//                     pkt.data[0], pkt.data[1], pkt.data[2], pkt.data[3],
//                     pkt.data[4], pkt.data[5], pkt.data[6], pkt.data[7],
//                     pkt.data[8], pkt.data[9], pkt.data[10], pkt.data[11],
//                     pkt.data[12], pkt.data[13], pkt.data[14], pkt.data[15]);
            av_packet_unref(&pkt);
            return boost::make_shared<foxglove_msgs::CompressedVideo>(video_msg);
        }
        return nullptr;
    }


private:
    AVCodec *codec_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVCodecContext *codec_context_ = nullptr;

    std::mutex mutex_;

    int64_t pts_ = 0;

    std::atomic<bool> received_ = false;
};

#endif //ROS1_WS_ENCODER_HPP
