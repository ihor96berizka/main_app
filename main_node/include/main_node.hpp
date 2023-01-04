#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "data_interface/msg/lidar_sample.hpp"
#include "data_interface/msg/distance_sensor_data.hpp"
#include "thread_safe_queue.hpp"
#include "solver.hpp"
#include "ros_data_provider.hpp"

using std::placeholders::_1;

class MainSwcNode : public rclcpp::Node
{
    using MyAdaptedType = data_interface::msg::LidarSample;
public:
    MainSwcNode();
    void init();
private:
    
    rclcpp::Subscription<MyAdaptedType>::SharedPtr subscription_;
    ThreadSafeQueue<MyAdaptedType> queue_;
    std::unique_ptr<std::jthread> processing_thread_;
    Solver::Solver solver_;
};

#endif // MAIN_NODE_HPP