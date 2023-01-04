#include "ros_data_provider.hpp"

#include <iostream>

RosDataProvider::RosDataProvider(ThreadSafeQueue<data_interface::msg::LidarSample>& queue) : queue_ref{queue}
{}

std::vector<Solver::DistanceSensorData> RosDataProvider::getSample()
{
    data_interface::msg::LidarSample raw_msg;
    queue_ref.pop(raw_msg);

    std::cout << "Repacking messages.....\n";

    std::vector<Solver::DistanceSensorData> converted_msgs;
    converted_msgs.reserve(Solver::kMaxNumberOfSamples);

    for (const auto& item: raw_msg.sensor_data)
    {
        converted_msgs.push_back({item.angle, item.distance});
    }
    std::cout << "Converted: " << converted_msgs.size() << " items\n";
    for (const auto& [angle, dist]: converted_msgs)
    {
        std::cout << "Angle: " << angle << " | dist: " << dist << std::endl;
    }
    std::cout << "Done.....\n";

    return converted_msgs;
}