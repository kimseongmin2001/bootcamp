#include "mjpeg_node/mjpeg_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MjpegNode>());
    rclcpp::shutdown();
    return 0;
}
