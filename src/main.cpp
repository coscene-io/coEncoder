//
// Created by fei on 24-12-5.
//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// #include "encoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class CoEncoder: public rclcpp::Node  {

public:
    CoEncoder() : Node("coencoder") {
        initEncoder();
//        auto topic_info = this->get_topic_names_and_types();
//
//        for (const auto &topic : topic_info) {
//            RCLCPP_ERROR(this->get_logger(), "Topic: %s, Type(s): %s",
//                        topic.first.c_str(),
//                        join_types(topic.second).c_str());
//        }
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            std::bind(&CoEncoder::image_callback, this, std::placeholders::_1)
        );

        yolo_subscription_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections",10,
            std::bind(&CoEncoder::yolo_callback, this, std::placeholders::_1)
        );
//        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//            "/camera/rgb/image_raw", 10,
//            [&](const sensor_msgs::msg::Image msg) {
//        		RCLCPP_ERROR(this->get_logger(), "get message form /camera/rgb/image_raw");
//                auto mat = convertToCvMat(msg);
//
//                cv::Mat yuv_img;
//                cv::cvtColor(mat, yuv_img, cv::COLOR_BGR2YUV_I420);
//                encoder_.send_frame(yuv_img);
//                auto frame = encoder_.encode_frame();
//                if (frame) {
//                    compressed_video_publisher_->publish(*frame);
//                }
//            }
//        );

        // Publisher for the /camera/h264 topic
        compressed_video_publisher_ = this->create_publisher<foxglove_msgs::msg::CompressedVideo>(
            "/camera/h264", 10);
    };
    ~CoEncoder() override {
    }

private:

    // H264Encoder encoder_;
//    std::string join_types(const std::vector<std::string> &types) {
//        std::string result;
//        for (const auto &type : types) {
//            if (!result.empty()) {
//                result += ", ";
//            }
//            result += type;
//        }
//        return result;
//    }

    void initEncoder(){
      	codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec_)
        {
            RCLCPP_ERROR(this->get_logger(), "can not find H.264 encoder");
            rclcpp::shutdown();
        }
        codec_context_ = avcodec_alloc_context3(codec_);
        if (!codec_context_)
        {
            RCLCPP_ERROR(this->get_logger(), "can not alloc encoder context");
            rclcpp::shutdown();
        }
        codec_context_->bit_rate = 400000;
        codec_context_->width = 640;
        codec_context_->height = 480;
        codec_context_->time_base = AVRational{1, 30};
        codec_context_->framerate = AVRational{30, 1};
        codec_context_->gop_size = 30;
        codec_context_->max_b_frames = 0;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;

        AVDictionary *codecOpts = nullptr;
        av_dict_set(&codecOpts, "tune", "zerolatency", 0);

        if (avcodec_open2(codec_context_, codec_, &codecOpts) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "can not open codec");
            rclcpp::shutdown();
        }

        frame_ = av_frame_alloc();
        frame_->format = codec_context_->pix_fmt;
        frame_->width = codec_context_->width;
        frame_->height = codec_context_->height;
        av_image_alloc(frame_->data, frame_->linesize, frame_->width,
                       frame_->height, codec_context_->pix_fmt, 32);

        packet_ = av_packet_alloc();

        sws_context_ = sws_getContext(frame_->width, frame_->height, AV_PIX_FMT_BGR24,
                                      frame_->width, frame_->height, AV_PIX_FMT_YUV420P,
                                      SWS_BILINEAR, nullptr, nullptr, nullptr);
    }

    void yolo_callback(yolo_msgs::msg::DetectionArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        detection_array_ = *msg;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto frame = convertToCvMat(*msg);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for(const auto &detection : detection_array_.detections) {
                	if (detection.class_name == "bird") {
                        int cx = detection.bbox.center.position.x;
                        int cy = detection.bbox.center.position.y;

                        int w = detection.bbox.size.x;
                        int h = detection.bbox.size.y;

    					cv::Point top_left(cx - w / 2, cy - h / 2);
    					cv::Point bottom_right(cx + w / 2, cy +h / 2);

                    	cv::rectangle(frame, top_left, bottom_right, cv::Scalar(0xeb, 0x63, 0x25), 2);
                	}
                }
            }

            const int stride[] = {static_cast<int>(frame.step[0])};
            sws_scale(sws_context_, &frame.data, stride, 0, frame.rows, frame_->data, frame_->linesize);

            frame_->pts = frame_counter_++;

            // 编码帧
            int ret = avcodec_send_frame(codec_context_, frame_);
            if (ret < 0)
            {
                char error_buffer[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, error_buffer, sizeof(error_buffer));
                RCLCPP_ERROR(this->get_logger(), "codec error: %s", error_buffer);
                return;
            }

            ret = avcodec_receive_packet(codec_context_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            {
                return;
            }
            else if (ret < 0)
            {
                char error_buffer[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, error_buffer, sizeof(error_buffer));
                RCLCPP_ERROR(this->get_logger(), "receive packet error: %s", error_buffer);
                return;
            }

            // 准备消息
            auto msg = foxglove_msgs::msg::CompressedVideo();
            msg.timestamp = this->now();
            msg.frame_id = "camera_frame";
            msg.data = std::vector<uint8_t>(packet_->data, packet_->data + packet_->size);
            msg.format = "h264";

            // 发布消息
            compressed_video_publisher_->publish(msg);

            av_packet_unref(packet_);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr yolo_subscription_;
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr compressed_video_publisher_;



    std::mutex mutex_;
    yolo_msgs::msg::DetectionArray detection_array_;

    AVCodec *codec_;
    AVCodecContext *codec_context_;
    AVFrame *frame_;
    AVPacket *packet_;
    SwsContext *sws_context_;
    uint64_t frame_counter_ = 0;

    cv::Mat convertToCvMat(const sensor_msgs::msg::Image &img_msg) {
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
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding type: %s", img_msg.encoding.c_str());
            return cv::Mat(); // Return an empty Mat in case of unsupported encoding
        }

        cv::Mat image(img_msg.height, img_msg.width, cv_type, const_cast<uchar *>(img_msg.data.data()), img_msg.step);
        if (img_msg.encoding == sensor_msgs::image_encodings::RGB8) {
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        }
        return image;

//        try {
//            // Use cv_bridge to convert ROS image to OpenCV Mat
//            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
//            return cv_ptr->image;
//        } catch (const cv_bridge::Exception &e) {
//            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//            return cv::Mat();  // Return an empty Mat in case of error
//        }
    }
};

int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("MAIN"), "Init");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CoEncoder>();
    RCLCPP_INFO(rclcpp::get_logger("MAIN"), "CoEncoder SPIN!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}