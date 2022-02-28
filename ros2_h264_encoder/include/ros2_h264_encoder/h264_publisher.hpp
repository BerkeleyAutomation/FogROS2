#ifndef H264_PUBLISHER_H
#define H264_PUBLISHER_H
// System Includes
#include <string>
#include <memory>

// External ROS Includes
#include "rclcpp/rclcpp.hpp"
#include "h264_msgs/msg/packet.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/simple_publisher_plugin.hpp"

// Project Includes
#include "ros2_h264_encoder/ros2_encoder.hpp"

class PacketPublisher : public image_transport::SimplePublisherPlugin<h264_msgs::msg::Packet> {
  public:
    rclcpp::Logger logger;
    PacketPublisher() : logger(rclcpp::get_logger("H264Publisher")) {
    }
  
    std::string getTransportName() const override {
      return "h264";
    }
  
  protected:
    void publish(const sensor_msgs::msg::Image& message,
               const PublishFn& publish_fn) const override;

    void advertiseImpl(
      rclcpp::Node* node,
      const std::string& base_topic,
      rmw_qos_profile_t custom_qos) override;

  private:
    std::shared_ptr<ROS2Encoder> encoder;
};
 

#endif