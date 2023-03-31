#include "fire_end_command_group.hpp"

namespace control
{

FireEndCommandGroup::FireEndCommandGroup(
    flywheel::FlywheelSubsystem *const flywheel,
    feeder::FeederSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand(drivers),
      fireCommand(flywheel, drivers) //,
    //   reverseFeedCommand(feeder, drivers)
{
    this->addSubsystemRequirement(flywheel);
    this->addSubsystemRequirement(feeder);
    this->comprisedCommandScheduler.registerSubsystem(flywheel);
    this->comprisedCommandScheduler.registerSubsystem(feeder);
}

void FireEndCommandGroup::initialize() {
    this->comprisedCommandScheduler.addCommand(&fireCommand);
    // this->comprisedCommandScheduler.addCommand(&reverseFeedCommand);
    flywheelDelayTimer.restart(FEEDER_DELAY_MS);
}

void FireEndCommandGroup::execute()
{
    if ( flywheelDelayTimer.execute()) {
            this->comprisedCommandScheduler.removeCommand(&fireCommand, false);
            // this->comprisedCommandScheduler.removeCommand(&reverseFeedCommand, false);
    }
    this->comprisedCommandScheduler.run();
}

void FireEndCommandGroup::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&fireCommand, interrupted);
    // this->comprisedCommandScheduler.removeCommand(&reverseFeedCommand, interrupted);
}

}  // namespace control



