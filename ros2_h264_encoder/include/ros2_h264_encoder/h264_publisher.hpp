#ifndef H264_PUBLISHER_H
#define H264_PUBLISHER_H
// System Includes
#include <string>
#include <memory>

// External ROS Includes
#include "rclcpp/rclcpp.hpp"
#include "h264_msgs/msg/packet.hpp"
#include "sensor_msgs/msg/image.hpp"

// Project Includes
#include "ros2_h264_encoder/ros2_encoder.hpp"

#define MAX_FRAME_RATE_SAMPLES 10

class PacketPublisher : public rclcpp::Node {
  public:
    PacketPublisher(const std::string & image_topic);

  private:
    int current_frames;
    rclcpp::Time first_received_timestamp;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr publisher;
    std::shared_ptr<ROS2Encoder> encoder;

    void on_image_received(sensor_msgs::msg::Image::SharedPtr msg);
};
 

#endif