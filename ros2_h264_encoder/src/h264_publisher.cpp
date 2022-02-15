// System Includes
#include <functional>
#include <cmath>

// Project Includes
#include "ros2_h264_encoder/h264_publisher.hpp"

PacketPublisher::PacketPublisher(const std::string & image_topic) : 
    Node("h264_publisher"), current_frames(0) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating H264 Image Transport Encoder for topic " << image_topic);
        image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(image_topic, 5, std::bind(&PacketPublisher::on_image_received, this, std::placeholders::_1));
        publisher = this->create_publisher<h264_msgs::msg::Packet>(image_topic + "_h264", 5);
        RCLCPP_INFO_STREAM(get_logger(), "Finished Creating H264 Image Transport Encoder for topic " << image_topic);
}

void PacketPublisher::on_image_received(sensor_msgs::msg::Image::SharedPtr msg) {
    if (current_frames < MAX_FRAME_RATE_SAMPLES) {
        if (current_frames == 0) {
            first_received_timestamp = this->now();
        }
        current_frames++;
        return;
    } else if (current_frames == MAX_FRAME_RATE_SAMPLES) {
        current_frames++;
        uint32_t fps = round(current_frames / ((this->now() - first_received_timestamp).nanoseconds() / 1e9));
        RCLCPP_INFO_STREAM(get_logger(), "Creating encoder with FPS: " << fps << ", image height: " << msg->height << ", image width: " << msg->width << " and encoding " << msg->encoding);
        encoder = std::make_shared<ROS2Encoder>(msg->width, msg->height, fps, msg->encoding);
        RCLCPP_INFO_STREAM(get_logger(), "Finished creating encoder with FPS: " << fps << ", image height: " << msg->height << ", image width: " << msg->width << " and encoding " << msg->encoding);
        return;
    }
    h264_msgs::msg::Packet packet = h264_msgs::msg::Packet();
    encoder->encode_image(msg, packet);
    publisher->publish(packet);
}