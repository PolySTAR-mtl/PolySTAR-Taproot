#include "feeder_position_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace feeder
{
void FeederPositionSubsystem::initialize()
{
    feederMotor.initialize();
    feederDesiredPos = feederMotor.getEncoderUnwrapped();
}

void FeederPositionSubsystem::refresh() {
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();
    updateController(feederDesiredPos, dt);

}

void FeederPositionSubsystem::updateController(float desiredPos, uint32_t dt) {
    float error = desiredPos - feederMotor.getEncoderUnwrapped();

    if (fabs(error) < DEGREE_TO_TICK) error = 0;

    float outputPID = feederPID.runControllerDerivateError(error, dt);
    float outputFF = feederFF.calculate(error);
    feederMotor.setDesiredOutput(outputPID + outputFF);
}

inline float FeederPositionSubsystem::getSetpoint() const {
    return feederDesiredPos;
};

void FeederPositionSubsystem::setSetpoint(float newAngle)  {
    feederDesiredPos = newAngle;
};

float FeederPositionSubsystem::getCurrentValue() const  {
    return feederMotor.getEncoderUnwrapped();
};

float FeederPositionSubsystem::getJamSetpointTolerance() const  {
    return jamChecker.getJamSetpointTolerance();
};

// Unused method, implemented for interface compatibility.
bool FeederPositionSubsystem::calibrateHere()  {
    return false;
};

bool FeederPositionSubsystem::isJammed()  {
    return jamChecker.check();
};

void FeederPositionSubsystem::clearJam()  {
    jamChecker.restart(); // Unsure, if bug look here
};

// Unused method, implemented for interface compatibility.
inline bool FeederPositionSubsystem::isCalibrated()  {
    return true;
};

inline bool FeederPositionSubsystem::isOnline()  {
    return feederMotor.isMotorOnline();
};

inline float FeederPositionSubsystem::getVelocity()  {
    return feederMotor.getShaftRPM();
};
}  // namespace feeder

}  // namespace control
