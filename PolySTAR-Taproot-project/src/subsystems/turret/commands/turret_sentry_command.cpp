#include "subsystems/turret/commands/turret_sentry_command.hpp"
#include "subsystems/sentry_general_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace turret
{
TurretSentryAimCommand::TurretSentryAimCommand(
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

void TurretSentryAimCommand::initialize()
{
    startMatchTimeout.restart(START_MATCH_WAIT_TIME);
}

void TurretSentryAimCommand::execute()
{
    if (!startMatchTimeout.isExpired())
    {
        turret->setAbsoluteOutputDegrees(0, 0);
        return;
    }
    operationMode.autoMode(this);
}

void TurretSentryAimCommand::end(bool)
{
    // Do nothing when switching back to manual aim,
    // ie leave current setpoints where they are.
}

bool TurretSentryAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

