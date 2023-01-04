#include "sensor_node.hpp"

int main(int argc, char * argv[])
{
    std::cout << "SENSOR APP.....\n";
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr sensor_node = std::make_shared<LidarSensorNode>();

    rclcpp::spin(sensor_node);
    rclcpp::shutdown();

    return 0;
}