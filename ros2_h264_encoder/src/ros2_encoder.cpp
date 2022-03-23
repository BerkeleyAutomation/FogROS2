// System Includes
#include <iostream>

// Project Includes
#include "ros2_h264_encoder/ros2_encoder.hpp"

const std::unordered_map<std::string, AVPixelFormat> ROS2Encoder::ROS_encoding_to_AV_Pixel_format = ROS2Encoder::create_ROS_encoding_to_AV_Pixel_format();

ROS2Encoder::ROS2Encoder(rclcpp::Logger logger) : pts(0), seq(0), logger(logger) {
    x264_param_default_preset(&params, "veryfast", "zerolatency");
    params.i_threads = 1;
    params.i_fps_den = 1;
}

ROS2Encoder::~ROS2Encoder() {
    x264_picture_clean(&input);
    x264_encoder_close(encoder);
    sws_freeContext(conversion_context);
}
    
bool ROS2Encoder::encode_image(const sensor_msgs::msg::Image &msg, h264_msgs::msg::Packet &packet) {
    if (!encoder) {
            RCLCPP_INFO_STREAM(logger, "Creating encoder with image height: " << msg.height << ", image width: " << msg.width << " and encoding " << msg.encoding);
            
            params.i_width = msg.width;
            params.i_height = msg.height;

            encoder = x264_encoder_open(&params);
            if (!encoder) {
                RCLCPP_WARN_STREAM(logger, "Could not open encoder!");
            }

            if(x264_picture_alloc(&input, X264_CSP_I420, params.i_width, params.i_height)){
                RCLCPP_WARN_STREAM(logger, "Cannot allocate x264 picure");
            }
            
            conversion_context = sws_getContext(msg.width, msg.height, ROS_encoding_to_AV_Pixel_format.at(msg.encoding), 
                    msg.width, msg.height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
            if (!conversion_context) {
                RCLCPP_WARN_STREAM(logger, "Failed to get conversion context!");
            }            
            RCLCPP_INFO_STREAM(logger, "Finished creating encoder with FPS image height: " << msg.height << ", image width: " << msg.width << " and encoding " << msg.encoding);
    }
    seq++;
    if (convert_image_to_h264(msg, &input)) {
        int frame_size = x264_encoder_encode(encoder, &nals, &i_nals, &input, &output);
        packet.seq = pts;
        pts++;
        if (frame_size >= 0) {
            for (int i = 0; i < i_nals; i++) {
                std::copy(nals[i].p_payload, nals[i].p_payload + nals[i].i_payload, std::back_inserter(packet.data));
            }
            return true;
        } else {
            RCLCPP_WARN_STREAM(logger, "Could not encode image!");
        }
    }
    return false;
}
    
bool ROS2Encoder::convert_image_to_h264(const sensor_msgs::msg::Image &msg, x264_picture_t* out) {
    int stride[3] = {static_cast<int>(msg.width) * 3, 0, 0};
    uint8_t *src[3]= {const_cast<uint8_t *>(&msg.data[0]), NULL, NULL};
    int returnedHeight = sws_scale(conversion_context, src, stride, 0, params.i_height, out->img.plane, out->img.i_stride);
    out->i_pts = pts;
    return returnedHeight == params.i_height;
}