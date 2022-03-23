// System Includes
#include <functional>
#include <cmath>

// Project Includes
#include "ros2_h264_encoder/h264_publisher.hpp"

void PacketPublisher::publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const {
    h264_msgs::msg::Packet packet = h264_msgs::msg::Packet();
    packet.header = message.header;
    packet.seq = encoder->get_seq();
    if (encoder->encode_image(message, packet)) {
        publish_fn(packet);
    }
}

void PacketPublisher::advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos) {
    SimplePublisherPlugin::advertiseImpl(node, base_topic, custom_qos);
    logger = node->get_logger();
    RCLCPP_INFO_STREAM(logger, "Started Encoder!");
    encoder = std::make_shared<ROS2Encoder>(node->get_logger());
}