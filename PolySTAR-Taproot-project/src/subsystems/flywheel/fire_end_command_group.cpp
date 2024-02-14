#include "fire_end_command_group.hpp"

namespace control
{
FireEndCommandGroup::FireEndCommandGroup(
    flywheel::FlywheelSubsystem *const flywheel,
    feeder::FeederPositionSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand(drivers),
      fireCommand(flywheel, drivers)
{
    this->addSubsystemRequirement(flywheel);
    this->addSubsystemRequirement(feeder);
    this->comprisedCommandScheduler.registerSubsystem(flywheel);
    this->comprisedCommandScheduler.registerSubsystem(feeder);
}

void FireEndCommandGroup::initialize() {
    this->comprisedCommandScheduler.addCommand(&fireCommand);
    flywheelDelayTimer.restart(FEEDER_DELAY_MS);
}

void FireEndCommandGroup::execute()
{
    if ( flywheelDelayTimer.execute()) {
            this->comprisedCommandScheduler.removeCommand(&fireCommand, false);
    }
    this->comprisedCommandScheduler.run();
}

void FireEndCommandGroup::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&fireCommand, interrupted);
}

}  // namespace control


