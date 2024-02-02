#include "turret_test_auto_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace turret
{
TurretTestAutoAimCommand::TurretTestAutoAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : GenericAutoAimCommand(turret, drivers)
{
}

void  TurretTestAutoAimCommand::execute()
{
    // Acquire setpoints received from CV over serial through CVHandler
    CVSerialData::Rx::TurretData turretData = drivers->cvHandler.getTurretData();
    float pitchSetpoint = turretData.pitchSetpoint*MRAD_TO_DEGREES;
    float yawSetpoint = turretData.yawSetpoint*MRAD_TO_DEGREES;

    turret->setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
}
}  // namespace turret
}  // namespace control

