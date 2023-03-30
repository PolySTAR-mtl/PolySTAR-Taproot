#include "fire_end_command_group.hpp"
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

FireEndCommandGroup::FireEndCommandGroup(
    FlywheelSubsystem *const flywheel,
    feeder::FeederSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand(drivers),
      fireCommand(flywheel, drivers),
      reverseFeedCommand(feeder, drivers)
{
    this->addSubsystemRequirement(flywheel);
    this->addSubsystemRequirement(feeder);
    this->comprisedCommandScheduler.registerSubsystem(flywheel);
    this->comprisedCommandScheduler.registerSubsystem(feeder);
}

void FireEndCommandGroup::initialize() {
    this->comprisedCommandScheduler.addCommand(&fireCommand);
    this->comprisedCommandScheduler.addCommand(&reverseFeedCommand);
    flywheelDelayTimer.restart(FEEDER_DELAY_MS);
}

void FireEndCommandGroup::execute()
{
    if ( flywheelDelayTimer.execute()) {
            this->comprisedCommandScheduler.removeCommand(&fireCommand, false);
            this->comprisedCommandScheduler.removeCommand(&reverseFeedCommand, false);
    }
    this->comprisedCommandScheduler.run();
}

void FireEndCommandGroup::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&fireCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&reverseFeedCommand, interrupted);
}

}  // namespace flywheel

}  // namespace control



