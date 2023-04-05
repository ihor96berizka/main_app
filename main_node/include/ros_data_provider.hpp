#ifndef ROS_DATA_PROVIDER_HPP
#define ROS_DATA_PROVIDER_HPP

#include "idataprovider.h"
#include "data_interface/msg/lidar_sample.hpp"
#include "thread_safe_queue.hpp"

class RosDataProvider : public Solver::IDataProvider
{
public:
    RosDataProvider(ThreadSafeQueue<data_interface::msg::LidarSample>& queue);
    std::vector<Solver::DistanceSensorData> getSample() override;

private:
    ThreadSafeQueue<data_interface::msg::LidarSample>& queue_ref;    
};

#endif // ROS_DATA_PROVIDER_HPP