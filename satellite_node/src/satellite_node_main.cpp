#include "satellite_node/satellite_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SatelliteNode>());
    rclcpp::shutdown();
    return 0;
}
