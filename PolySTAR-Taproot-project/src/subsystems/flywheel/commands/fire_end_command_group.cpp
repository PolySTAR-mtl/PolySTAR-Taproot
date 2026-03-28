#include "fire_end_command_group.hpp"

namespace control
{
FireEndCommandGroup::FireEndCommandGroup(
    flywheel::FlywheelSubsystem *const flywheel,
    feeder::FeederPositionSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand{drivers},
      fireCommand_{flywheel, drivers},
      flywheelDelayTimer_{}
{
    this->addSubsystemRequirement(flywheel);
    this->addSubsystemRequirement(feeder);
    this->comprisedCommandScheduler.registerSubsystem(flywheel);
    this->comprisedCommandScheduler.registerSubsystem(feeder);
}

void FireEndCommandGroup::initialize() {
    this->comprisedCommandScheduler.addCommand(&fireCommand_);
    flywheelDelayTimer_.restart(flywheel::FEEDER_DELAY_MS);
}

void FireEndCommandGroup::execute()
{
    if (flywheelDelayTimer_.execute()) {
        this->comprisedCommandScheduler.removeCommand(&fireCommand_, false);
    }
    this->comprisedCommandScheduler.run();
}

void FireEndCommandGroup::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&fireCommand_, interrupted);
}

bool FireEndCommandGroup::isFinished() const 
{
    return false;
}

const char* FireEndCommandGroup::getName() const
{
    return "fire end command group"; 
}

}  // namespace control


