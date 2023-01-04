#include "main_node.hpp"
#include "ros_data_provider.hpp"

MainSwcNode::MainSwcNode(): Node("main_swc_node")
{
}

void MainSwcNode::init()
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

    auto dataProvider = std::make_unique<RosDataProvider>(queue_);
    solver_.init(std::move(dataProvider));

    processing_thread_ = std::make_unique<std::jthread>([this](std::stop_token stoken)
    {
        while (!stoken.stop_requested())
        {
            int angle = solver_.calculateHeadingAngle();
            std::cout << "Safe angle: " << angle << std::endl;
            /*
            * send angle to actuator
            */ 
        }
    });
}
