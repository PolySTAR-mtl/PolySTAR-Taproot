#include "feeder_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace feeder
{
void FeederSubsystem::initialize()
{
    feederMotor.initialize();
}

void FeederSubsystem::refresh() {
    updatePosPid(feederDesiredPos);
}

void FeederSubsystem::updatePosPid(float desiredPos) {
    // sets position with pid
}

inline float FeederSubsystem::getSetpoint() const {
    return feederDesiredPos;
};

void FeederSubsystem::setSetpoint(float newAngle)  {
    feederDesiredPos = newAngle;
};

float FeederSubsystem::getCurrentValue() const  {
    return feederMotor.getEncoderWrapped();
};

float FeederSubsystem::getJamSetpointTolerance() const  {
    return JAM_SETPOINT_POS_TOLERANCE_DEG;
};

// TODO
bool FeederSubsystem::calibrateHere()  {
    
};

bool FeederSubsystem::isJammed()  {
    return jamChecker.check();
};

void FeederSubsystem::clearJam()  {
    // contents to be determined
};

// TODO
inline bool FeederSubsystem::isCalibrated()  {
    
};

inline bool FeederSubsystem::isOnline()  {
    return feederMotor.isMotorOnline();
};

inline float FeederSubsystem::getVelocity()  {
    // yet to be used
    return NULL;
};
}  // namespace feeder

}  // namespace control
