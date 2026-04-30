#include "vlm_node/vlm_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VlmNode>());
    rclcpp::shutdown();
    return 0;
}
