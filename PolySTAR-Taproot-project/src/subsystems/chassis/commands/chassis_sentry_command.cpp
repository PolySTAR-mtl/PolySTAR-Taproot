#include "chassis_sentry_command.hpp"
#include "subsystems/sentry_general_constants.hpp"

namespace control::chassis 
{

ChassisSentryDriveCommand::ChassisSentryDriveCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : chassis(chassis),
    drivers(drivers),
    startMatchTimeout(0)
{
    if (chassis == nullptr)
    {
        return;
    }

    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}
void ChassisSentryDriveCommand::initialize() 
{
    startMatchTimeout.restart(START_MATCH_WAIT_TIME);
}

void ChassisSentryDriveCommand::execute()
{
    operationMode.autoMode(this);
}

void ChassisSentryDriveCommand::end(bool) 
{
    chassis->setTargetOutput(0,0,0);
}

bool ChassisSentryDriveCommand::isFinished() const { return false; }

} // namespace control::chassis