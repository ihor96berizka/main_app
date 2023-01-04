#include "main_node.hpp"

MainSwcNode::MainSwcNode(): Node("main_swc_node")
{
    subscription_ = this->create_subscription<MyAdaptedType>(
    "lidar", 10, [this](const MyAdaptedType& message)
    {
        RCLCPP_INFO(this->get_logger(), "Received from sensor items: %lu", message.sensor_data.size());
        for (const auto& item: message.sensor_data)
        {
            RCLCPP_INFO(this->get_logger(), "[%d, %f]", item.angle, item.distance);
        }
        queue_.push(message);
    });
}