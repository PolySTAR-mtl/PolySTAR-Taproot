#ifndef CASCADED_PID_HPP_
#define CASCADED_PID_HPP_

#include <cstdint>
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "algorithms_constants.hpp"
#include "../subsystems/turret/turret_constants.hpp"


namespace src
{
class Drivers;
}

namespace src
{
namespace algorithms
{

class CascadedPid
{
    // constructor
    CascadedPid(Drivers* drivers): 
    drivers(drivers), 
    innerPid(INNER_PID_CONFIG),
    outerPid(OUTER_PID_CONFIG)
    {
    };
    ~CascadedPid() = default;
    CascadedPid(const CascadedPid &other) = delete;
    CascadedPid &operator=(const CascadedPid &other) = delete;

public:
    void updateYaw(float desiredPos, float currentPos, float currentRpm, float deltaTime);
    void updatePitch(tap::motor::DjiMotor* const motor, float desiredPos, uint32_t dt);
protected:
   
 // PID controller for position feedback from motor
    tap::algorithms::SmoothPid innerPid;
    tap::algorithms::SmoothPid outerPid;

    src::Drivers *drivers;
};

}  // namespace algorithms

}  // namespace src

#endif  // CASCADED_PID_HPP_
