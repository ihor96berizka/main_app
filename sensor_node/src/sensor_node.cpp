#include "sensor_node.hpp"

LidarSensorNode::LidarSensorNode(): Node("lidar_sensor_node")
{
    publisher_ =
        this->create_publisher<data_interface::msg::LidarSample>("lidar", 10);

    auto publish_msg = [this](){
        std::cout << "Publishing sample...." << std::endl;
    /**
     * read data from lidar sensor and pack inside message.
     * for now - only dummy impl exists
    */
    auto message = data_interface::msg::LidarSample();
        for (int32_t idx = 0; idx < 12; ++idx)
        {
           
            data_interface::msg::DistanceSensorData data;
            data.angle = idx;
            data.distance = 2 * idx + 1;
            message.sensor_data.push_back(data);
        }

        this->publisher_->publish(message);
    };
    
    timer_ = this->create_wall_timer(3s, publish_msg);
}