/**
 * @file idataprovider.h
 * Contains common functions and definition of data interface.
*/
#ifndef IDATAPROVIDER_H
#define IDATAPROVIDER_H

#include <vector>
#include <cmath>

namespace Solver
{
/***
 * @brief Specifies maximum number of samples that sensor can provide 
 * at one time.
*/
constexpr size_t kMaxNumberOfSamples{360};

/**
 * @brief Converts degrees to radians.
 * 
 * @param degrees - value to be converted to radians.
 * 
 * @return Converted value.
*/
constexpr double DegreesToRadians(double degrees) {
    return degrees * (M_PI / 180);
}

/**
 * @brief Converts radians to degrees.
 * 
 * @param radians - value to be converted to degrees.
 * 
 * @return Converted value.
*/
constexpr double RadiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

/**
 * @brief Struct which represents data from sensor.
 *  
*/
struct DistanceSensorData
{
    /**
     * @brief angle in degrees.
    */
    double angle;

    /**
     * @brief distance to obstacle in meters.
    */
    double distance;
};

/**
 * @brief Interface for data provider.
 * Client should implement this interface in order to 
 * pass sensors data to Solver.
*/
class IDataProvider
{
public:

    /**
     * @brief Interface method for fethcing data sample from sensor.
     * 
     * @return Packed data from sensor into DistanceSensorData and returns.
    */
    virtual std::vector<DistanceSensorData> getSample() = 0;
};

}//  namespace Solver
#endif // IDATAPROVIDER_H
