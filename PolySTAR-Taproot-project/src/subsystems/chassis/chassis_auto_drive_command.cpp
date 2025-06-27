#include "chassis_auto_drive_command.hpp"
#include "subsystems/sentry_general_constants.hpp"

namespace control
{
namespace chassis
{
ChassisAutoDriveCommand::ChassisAutoDriveCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : GenericAutoDriveCommand(chassis, drivers),
      startMatchTimeout(0)
{
    startMatchTimeout.stop();
}

void ChassisAutoDriveCommand::initialize() 
{
    startMatchTimeout.restart(START_MATCH_WAIT_TIME);
}

void ChassisAutoDriveCommand::execute()
{
    if (!startMatchTimeout.isExpired())
    {
        chassis->setTargetOutput(0, 0, 0);
        return;
    }
    drivers->leds.set(tap::gpio::Leds::A, true);
    GenericAutoDriveCommand::execute();
}

}  // namespace chassis
}  // namespace control

