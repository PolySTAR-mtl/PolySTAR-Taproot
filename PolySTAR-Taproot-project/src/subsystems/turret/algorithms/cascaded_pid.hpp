#ifndef CASCADED_PID_HPP_
#define CASCADED_PID_HPP_

#include <cstdint>
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "algorithms_constants.hpp"
#include "../turret_constants.hpp"


namespace turret
{
namespace algorithms
{

class CascadedPid
{
public:
    CascadedPid(): 
    innerPid(INNER_PID_CONFIG),
    outerPid(OUTER_PID_CONFIG)
    {
    };
    ~CascadedPid() = default;
    CascadedPid(const CascadedPid &other) = delete;
    CascadedPid &operator=(const CascadedPid &other) = delete;

    void updateYaw(float posError, float currentRpm, float dt);
    void updatePitch(tap::motor::DjiMotor* const motor, float desiredPos, uint32_t dt);
    void updatePitch(int64_t angleError, int16_t currentRpm, uint32_t dt);
    float getOutput() { return innerPid.getOutput(); }
    /*
    float getYawOutput() { return yawOutput;}
    */
    float getPitchOutput() { return pitchOutput;}
protected:
   
 // PID controller for position feedback from motor
    tap::algorithms::SmoothPid innerPid;
    tap::algorithms::SmoothPid outerPid;

 // PID output for pitch and yaw
    float yawOutput = 0.0f;
    float pitchOutput = 0.0f;

};

}  // namespace algorithms

}  // namespace turret

#endif  // CASCADED_PID_HPP_
