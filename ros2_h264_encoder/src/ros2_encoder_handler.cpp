// System Includes
#include <memory>

// External ROS Includes
#include "rclcpp/rclcpp.hpp"

// Project Includes
#include "ros2_h264_encoder/h264_publisher.hpp"

class PacketPublisherNode : public rclcpp::Node {
    public:
        PacketPublisherNode() : Node("test"){}
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::string topic = "/camera/image_raw";
    rclcpp::spin(std::make_shared<PacketPublisherNode>());
    rclcpp::shutdown();
    return 0;
}