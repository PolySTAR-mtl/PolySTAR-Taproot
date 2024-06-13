#ifndef CASCADED_PID_HPP_
#define CASCADED_PID_HPP_

#include <cstdint>
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "../turret_constants.hpp"


namespace turret
{
namespace algorithms
{

/* Cascaded PID controller for turret control
   Outer PID takes position error as input and controls RPM setpoint
   Inner PID takes RPM error as input and controls motor voltage
   Normally velocity controller is a PI controller and position controller is a PD controller
   Note on maximum values : 
    positionConfig.maxOutput is the maximum RPM of the turret
    velocityConfig.maxOutput is the maximum voltage sent to the motor */
class CascadedPid
{
public:
    CascadedPid(const tap::algorithms::SmoothPidConfig &velocityConfig,const tap::algorithms::SmoothPidConfig &positionConfig );

    // Calculate output from position error and current RPM
    void update(float positionError, float currentRpm, float dt);
    // Test velocity loop, used for tuning
    void testInnerLoop(float positionError, float currentRpm, float dt, float desiredRpm, float threshold);
    // Get current output of controller
    float getOutput() { return output; }

    enum Parameter {
        P, I, D,
        MAX_OUTPUT,
        MAX_I_CUMULATIVE,
        ERR_DEADZONE
    };

    enum Controller {
        VELOCITY,
        POSITION
    };

    // Update controller parameters during runtime
    void setParameter(Controller controller, Parameter param, float value);

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
