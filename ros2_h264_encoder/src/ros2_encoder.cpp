// System Includes
#include <iostream>

// Project Includes
#include "ros2_h264_encoder/ros2_encoder.hpp"

const std::unordered_map<std::string, AVPixelFormat> ROS2Encoder::ROS_encoding_to_AV_Pixel_format = ROS2Encoder::create_ROS_encoding_to_AV_Pixel_format();

ROS2Encoder::ROS2Encoder(const int width, const int height, const uint32_t fps, 
                        const std::string & img_encoding) : pts(0) {
    x264_param_default_preset(&params, "veryfast", "zerolatency");
    params.i_threads = 1;
    params.i_width = width;
    params.i_height = height;
    params.i_fps_num = fps;
    params.i_fps_den = 1;

    encoder = x264_encoder_open(&params);
    if (!encoder) {
        std::cout << "Could not open encoder!" << std::endl;
    }

    if(x264_picture_alloc(&input, X264_CSP_I420, params.i_width, params.i_height)){
        std::cout << "Cannot allocate x264 picure" << std::endl;
    }
    
    conversion_context = sws_getContext(width, height, ROS_encoding_to_AV_Pixel_format.at(img_encoding), 
            width, height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    if (!conversion_context) {
        std::cout << "FAILED TO GET CONTEXT!" << std::endl;
    }
}

ROS2Encoder::~ROS2Encoder() {
    x264_picture_clean(&input);
    x264_encoder_close(encoder);
    sws_freeContext(conversion_context);
}
    
bool ROS2Encoder::encode_image(sensor_msgs::msg::Image::SharedPtr &msg, h264_msgs::msg::Packet &packet) {
    std::cout << "Received Picture" << std::endl;
    if (convert_image_to_h264(msg, &input)) {
        std::cout << "Converted image!" << std::endl;
        int frame_size = x264_encoder_encode(encoder, &nals, &i_nals, &input, &output);
        pts++;
        if (frame_size >= 0) {
            std::cout << "Successful encoding!" << std::endl;
            for (int i = 0; i < i_nals; i++) {
                std::copy(nals[i].p_payload, nals[i].p_payload + nals[i].i_payload, std::back_inserter(packet.data));
            }
            return true;
        } else {
            std::cout << "Encoder failed!" << std::endl;
        }
    }
    return false;
}
    
bool ROS2Encoder::convert_image_to_h264(sensor_msgs::msg::Image::SharedPtr &msg, x264_picture_t* out) {
    int stride[3] = {msg->width * 3, 0, 0};
    uint8_t *src[3]= {&msg->data[0], NULL, NULL};
    int returnedHeight = sws_scale(conversion_context, src, stride, 0, params.i_height, out->img.plane, out->img.i_stride);
    out->i_pts = pts;
    return returnedHeight == params.i_height;
}