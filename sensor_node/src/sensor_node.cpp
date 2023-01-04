#include "sensor_node.hpp"

LidarSensorNode::LidarSensorNode(): Node("lidar_sensor_node")
{
    publisher_ =
        this->create_publisher<data_interface::msg::LidarSample>("lidar", 10);

    auto publish_msg = [this](){
        auto message = data_interface::msg::LidarSample();

        data_interface::msg::DistanceSensorData data;
        data.angle = 1;
        data.distance = 2.3;
        message.sensor_data.push_back(data);
        std::cout << "Publishing sample: [" << data.angle << " | " << data.distance << "]" << std::endl;
        this->publisher_->publish(message);
        };
    
    timer_ = this->create_wall_timer(1s, publish_msg);
}