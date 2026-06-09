#include "chassis_sentry_command.hpp"
#include "subsystems/sentry_general_constants.hpp"


namespace control::chassis 
{
    ChassisSentryCommand::ChassisSentryCommand(
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
    void ChassisSentryCommand::initialize() 
    {
        startMatchTimeout.restart(START_MATCH_WAIT_TIME);
    }

    void ChassisSentryCommand::execute()
    {
        operationMode.autoMode(this);
    }

    void ChassisSentryCommand::end(bool) 
    {
        chassis->setTargetOutput(0,0,0);
    }

    bool ChassisSentryCommand::isFinished() const { return false; }

} // namespace control::chassis