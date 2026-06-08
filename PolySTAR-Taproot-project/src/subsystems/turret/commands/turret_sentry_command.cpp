#include "subsystems/turret/commands/turret_sentry_command.hpp"
#include "subsystems/sentry_general_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace turret
{
SentryAimCommand::SentryAimCommand(
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

void SentryAimCommand::initialize()
{
    startMatchTimeout.restart(START_MATCH_WAIT_TIME);
}

void SentryAimCommand::execute()
{
    if (!startMatchTimeout.isExpired())
    {
        turret->setAbsoluteOutputDegrees(0, 0);
        return;
    }
    // Acquire setpoints received from CV over serial through CVHandler
    CVSerialData::Rx::TurretData turretData = drivers->cvHandler.getTurretData();
    float pitchSetpoint = turretData.pitchSetpoint*autoAttributes->MRAD_TO_DEGREES;
    float yawSetpoint = turretData.yawSetpoint*autoAttributes->MRAD_TO_DEGREES;

    turret->setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
}

void SentryAimCommand::end(bool)
{
    // Do nothing when switching back to manual aim,
    // ie leave current setpoints where they are.
}

bool SentryAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

