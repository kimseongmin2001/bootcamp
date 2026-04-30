#include "udp_node/udp_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpNode>());
    rclcpp::shutdown();
    return 0;
}
