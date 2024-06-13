#include "cascaded_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

namespace turret
{
namespace algorithms
{

CascadedPid::CascadedPid(const tap::algorithms::SmoothPidConfig& positionConfig,
                         const tap::algorithms::SmoothPidConfig& velocityConfig)
    : positionController(positionConfig), velocityController(velocityConfig), output(0.0f)
{
}

// Run the cascaded PID controller. Updates output based on the position error and current RPM.
void CascadedPid::update(float positionError, float currentRPM, float dt) {
    // Obtain Desired RPM from position error
    positionController.runController(positionError, currentRPM, dt);
    float desiredRPM = positionController.getOutput();

    // Obtain motor voltage from desired RPM
    float rateError = desiredRPM - currentRPM;
    velocityController.runController(rateError, 0, dt);
    output = velocityController.getOutput();
}

// Method used for tuning the inner (velocity) loop of the cascaded PID controller
// Control only applies inner control loop and a fixed velocity setpoint
void CascadedPid::testInnerLoop(float positionError, float currentRPM, float dt, float desiredRPM, float threshold) {
    
    if (abs(positionError) < threshold) {
        desiredRPM = 0;
    } else {
        desiredRPM = tap::algorithms::getSign(positionError) * desiredRPM;
    }
    
    float rateError = desiredRPM - currentRPM;
    velocityController.runController(rateError, 0, dt);
    output = velocityController.getOutput();
}

// Not sure whether to keep this
// This is to avoid having 12 different setters for controller parameters
void CascadedPid::setParameter(Controller controller, Parameter param, float value) {
    if (controller == VELOCITY) {
        switch (param)
        {
        case P:
            velocityController.setP(value);
            break;
        case I:
            velocityController.setI(value);
            break;
        case D:
            velocityController.setD(value);
            break;
        case MAX_OUTPUT:
            velocityController.setMaxOutput(value);
            break;
        case MAX_I_CUMULATIVE:
            velocityController.setMaxICumulative(value);
            break;
        case ERR_DEADZONE:
            velocityController.setErrDeadzone(value);
            break;
        default:
        break;
        }
    } else if (controller == POSITION) {
        switch (param)
        {
        case P:
            positionController.setP(value);
            break;
        case I:
            positionController.setI(value);
            break;
        case D:
            positionController.setD(value);
            break;
        case MAX_OUTPUT:
            positionController.setMaxOutput(value);
            break;
        case MAX_I_CUMULATIVE:  
            positionController.setMaxICumulative(value);
            break;
        case ERR_DEADZONE:
            positionController.setErrDeadzone(value);
            break;
        default:
        break;
        }
    }
}

}  // namespace algorithms

}  // namespace turret
