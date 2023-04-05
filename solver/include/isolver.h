#ifndef ISOLVER_H
#define ISOLVER_H

#include "idataprovider.h"

#include <memory>
#include <string>
#include <vector>

namespace Solver
{
/**
 * @brief Describes obstacle with a set of parameters.
*/
struct Obstacle
{
    /**
     * @brief Contains distance to obstacles(in meters) 
    */
    std::vector<double> distances;

    /**
     * @brief Contains angles(in degrees) in which direction distance is measured.
    */
    std::vector<double> angles;

    /**
     * @brief Average distance to obstacle(in meters).
     * Is needed for further calculations.
    */
    double averageDistance;

    /**
     * @brief Average angle(in rads) occupied by obstacle.
    */
    double averageAngle;

    /**
     * @brief Power coefficient. Is used to ensure that component
     * of repulsive field fully embaraces obstacle and its
     * magnitude is comparable to magnitude of attractive field.
    */
    double a;
};

/**
 * @brief Contains parameters to configure ISolver.
*/
struct SolverParams
{
    /**
     * @brief Minimum distance to object. If object is located at closer
     * distance - it will be considered as obstacles. Measured in meters.
    */
    static constexpr double _thresholdDistance = 2;

    /**
     * @brief Robot width in meters.
    */
    static constexpr double _w_robot = 0.25;

    /**
     * @brief Maximum range of distance sensor. Measured in meters.
    */
    static constexpr double _distance_sensor_range = 10.0;

    /**
     * @brief Heading angle which points to desired direction. Measured in degrees.
    */
    static constexpr double _teta_goal = (60);

    /**
     * @brief Scale parameter for attractive field.
    */
    static constexpr double _gamma = 0.5; // see eq (11)
};

struct Forces
{
    std::vector<DistanceSensorData> repulsiveFieldData;
    std::vector<DistanceSensorData> attrFieldData;
    std::vector<DistanceSensorData> totalFieldData;
};

/*
* Usage: Create instance of Solver.
* Flow:
*   init(provider)
*   calculateHeadingAngle()
* calculateHeadingAngle() will wait for new chuck of data and then perform calculations.
* It should be called in a working loop in user code.
*/
class ISolver
{
public:
    void init(std::unique_ptr<IDataProvider> dataProvider);
    std::vector<DistanceSensorData> getSensorData();
    Forces getForces();
    virtual int calculateHeadingAngle() = 0;

protected:
    std::vector<DistanceSensorData> _distanceSensorData;
    Forces _forces;
    std::unique_ptr<IDataProvider> _dataProvider;

    std::vector<Obstacle> findObstacles();
    void enlargeObstacles(std::vector<Obstacle>& obstacles, const double w_robot);
    void calculateObstaclesAverages(std::vector<Obstacle> &obstacles);
    virtual std::vector<DistanceSensorData> calculateRepulsiveField() = 0;
    virtual std::vector<DistanceSensorData> calculateAttractiveField() = 0;
    void calculateForces();
    std::vector<DistanceSensorData> calculateTotalField(const std::vector<DistanceSensorData>& repulsive,
                                                        const std::vector<DistanceSensorData>& attractive);
};
}
#endif // ISOLVER_H
