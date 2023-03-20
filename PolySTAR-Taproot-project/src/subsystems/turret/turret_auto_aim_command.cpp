#include "turret_auto_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace turret
{
TurretAutoAimCommand::TurretAutoAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : turret(turret),
      drivers(drivers)
{
    if (turret == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(turret));
}

void  TurretAutoAimCommand::initialize() {}

void  TurretAutoAimCommand::execute()
{
    const float MRAD_TO_DEGREES = 0.0572958;

    // Acquire setpoints received from CV over serial through CVHandler
    CVSerialData::Rx::TurretData turretData = drivers->cvHandler.getTurretData();
    float pitch = turretData.pitchSetpoint*MRAD_TO_DEGREES;
    float yaw = turretData.yawSetpoint*MRAD_TO_DEGREES;

    turret->setAbsoluteOutput(yaw, pitch);
}

void  TurretAutoAimCommand::end(bool) {
    // Do nothing when switching back to manual aim, 
    // ie leave current setpoints where they are.
}

bool  TurretAutoAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

