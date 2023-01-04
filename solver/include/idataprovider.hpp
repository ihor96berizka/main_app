#ifndef IDATAPROVIDER_HPP
#define IDATAPROVIDER_HPP

#include <vector>
namespace Solver
{
constexpr size_t kMaxNumberOfSamples{360};

struct DistanceSensorData
{
    int angle;
    double distance;
};

class IDataProvider
{
public:
/*
*   @brief Fetch data from some source.
*   This should be blocking call.
*/
    virtual std::vector<DistanceSensorData> getSample() = 0;
};

}//  namespace Solver
#endif // IDATAPROVIDER_HPP
