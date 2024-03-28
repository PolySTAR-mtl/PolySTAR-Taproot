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

/* Cascaded PID controller for turret control
   First PID takes position error as input and outputs RPM setpoint
   Second PID takes RPM error as input and outputs motor voltage
   Normally velocity controller is a PI controller and position controller is a PD controller
   Notes on maximum values : 
    positionConfig.maxOutput is the maximum RPM output of the outer PID controller
    velocityConfig.maxOutput is the maximum voltage output of the inner PID controller */
class CascadedPid
{
public:
    CascadedPid(const tap::algorithms::SmoothPidConfig &velocityConfig,const tap::algorithms::SmoothPidConfig &positionConfig );

    // Calculate output from position error and current RPM
    void update(float positionError, float currentRpm, float dt);
    // Get current output of controller
    float getOutput() { return output; }

    // PID configuration setters
    inline void setVelocityP(float p) { velocityController.setP(p); }
    inline void setVelocityI(float i) { velocityController.setI(i); }
    inline void setVelocityD(float d) { velocityController.setD(d); }

    inline void setPositionP(float p) { positionController.setP(p); }
    inline void setPositionI(float i) { positionController.setI(i); }
    inline void setPositionD(float d) { positionController.setD(d); }

    inline void setVelocityMaxICumulative(float maxICumulative) { velocityController.setMaxICumulative(maxICumulative); }
    inline void setVelocityMaxOutput(float maxOutput)           { velocityController.setMaxOutput(maxOutput); }
    inline void setVelocityErrDeadzone(float errDeadzone)       { velocityController.setErrDeadzone(errDeadzone); }

    inline void setPositionMaxICumulative(float maxICumulative) { positionController.setMaxICumulative(maxICumulative); }
    inline void setPositionMaxOutput(float maxOutput)           { positionController.setMaxOutput(maxOutput); }
    inline void setPositionErrDeadzone(float errDeadzone)       { positionController.setErrDeadzone(errDeadzone); }

protected:
   
    // Inner and outer control loops
    tap::algorithms::SmoothPid positionController;
    tap::algorithms::SmoothPid velocityController;

    // Voltage Output
    float output;
};

}  // namespace algorithms

}  // namespace turret

#endif  // CASCADED_PID_HPP_
