#include "fire_command_group.hpp"
#include "tap/control/comprised_command.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_subsystem.hpp"
#include "flywheel_fire_command.hpp"
#include "flywheel_constants.hpp"
#include "subsystems/feeder/feeder_feed_command.hpp"
namespace control
{
namespace flywheel
{

FireCommandGroup::FireCommandGroup(
    FlywheelSubsystem *const flywheel,
    feeder::FeederSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand(drivers),
      fireCommand(flywheel, drivers),
      feedCommand(feeder, drivers),
      fireEndCommand(flywheel,feeder,drivers),
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

}  // namespace flywheel

}  // namespace control



