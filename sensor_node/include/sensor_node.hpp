#ifndef SENSOR_NODE_HPP
#define SENSOR_NODE_HPP

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "data_interface/msg/lidar_sample.hpp"
#include "data_interface/msg/distance_sensor_data.hpp"

using namespace std::chrono_literals;

class LidarSensorNode : public rclcpp::Node
{
public:
    LidarSensorNode();
private:
    rclcpp::Publisher<data_interface::msg::LidarSample>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


#endif // SENSOR_NODE_HPP