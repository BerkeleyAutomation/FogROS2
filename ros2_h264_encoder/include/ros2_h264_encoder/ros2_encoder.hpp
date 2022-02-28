#ifndef ROS2_ENCODER_H
#define ROS2_ENCODER_H
// System Includes
#include <string>
#include <unordered_map>
#include <chrono>

// External ROS Includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "h264_msgs/msg/packet.hpp"

// External Includes
extern "C"
{
#include <x264.h>
#include <libswscale/swscale.h>
}

#define MAX_FRAME_RATE_SAMPLES 10

class ROS2Encoder {
public:
    ROS2Encoder() : logger(rclcpp::get_logger("H264PublisherEncoder")) {}
    ROS2Encoder(rclcpp::Logger logger);
    ~ROS2Encoder();
    bool encode_image(const sensor_msgs::msg::Image &msg, h264_msgs::msg::Packet &packet);
    bool convert_image_to_h264(const sensor_msgs::msg::Image &msg, x264_picture_t* out);
    int get_seq() {
        return seq;
    }

private:
    // Encoder variables
    x264_t* encoder;
    x264_param_t params;

    x264_picture_t input;
    x264_picture_t output;
    x264_nal_t* nals;
    int i_nals;
    int64_t pts;

    int seq;
    
    // Conversion
    struct SwsContext* conversion_context;

    static std::unordered_map<std::string, AVPixelFormat> create_ROS_encoding_to_AV_Pixel_format() {
        std::unordered_map<std::string, AVPixelFormat> formats;
        formats[sensor_msgs::image_encodings::BGR16] = AV_PIX_FMT_BGR48;
        formats[sensor_msgs::image_encodings::BGR8] = AV_PIX_FMT_BGR24;
        formats[sensor_msgs::image_encodings::BGRA16] = AV_PIX_FMT_BGRA64;
        formats[sensor_msgs::image_encodings::BGRA8] = AV_PIX_FMT_BGRA;
        formats[sensor_msgs::image_encodings::MONO16] = AV_PIX_FMT_GRAY16;
        formats[sensor_msgs::image_encodings::MONO8] = AV_PIX_FMT_GRAY8;
        formats[sensor_msgs::image_encodings::RGB16] = AV_PIX_FMT_RGB48;
        formats[sensor_msgs::image_encodings::RGB8] = AV_PIX_FMT_RGB24;        
        formats[sensor_msgs::image_encodings::RGBA16] = AV_PIX_FMT_RGBA64;
        formats[sensor_msgs::image_encodings::RGBA8] = AV_PIX_FMT_RGBA;
        return formats;
    }

    static const std::unordered_map<std::string, AVPixelFormat> ROS_encoding_to_AV_Pixel_format;

    // ROS
    rclcpp::Logger logger;
};

#endif

