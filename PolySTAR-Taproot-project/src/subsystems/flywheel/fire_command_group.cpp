#include "fire_command_group.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_subsystem.hpp"
#include "flywheel_fire_command.hpp"
#include "flywheel_constants.hpp"

namespace control
{
FireCommandGroup::FireCommandGroup(
    flywheel::FlywheelSubsystem *const flywheel,
    feeder::FeederPositionSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand(drivers),
      fireCommand(flywheel, drivers),
      feedCommand(feeder, drivers),
      feederIsFeeding(false)
{
    this->addSubsystemRequirement(flywheel);
    this->addSubsystemRequirement(feeder);
    this->comprisedCommandScheduler.registerSubsystem(flywheel);
    this->comprisedCommandScheduler.registerSubsystem(feeder);
}

void FireCommandGroup::initialize() {
    this->comprisedCommandScheduler.addCommand(&fireCommand);
    feederDelayTimer.restart(FEEDER_DELAY_MS);
    feederIsFeeding = false;
}

void FireCommandGroup::execute()
{
    if ( feederIsFeeding == false && feederDelayTimer.execute())
        {
            comprisedCommandScheduler.addCommand(&feedCommand);
            feederIsFeeding = true;
        }
    this->comprisedCommandScheduler.run();
}

void FireCommandGroup::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&feedCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&fireCommand, interrupted);
}

}  // namespace control



