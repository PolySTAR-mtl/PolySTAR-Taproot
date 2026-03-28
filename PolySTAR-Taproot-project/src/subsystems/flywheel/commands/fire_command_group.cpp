#include "fire_command_group.hpp"

#include "control/drivers/drivers.hpp"
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "flywheel_fire_commands.hpp"
#include "subsystems/flywheel/config/flywheel_constants.hpp"

namespace control
{
FireCommandGroup::FireCommandGroup(
    flywheel::FlywheelSubsystem *const flywheel,
    feeder::FeederPositionSubsystem *const feeder,
    src::Drivers* drivers)
    : tap::control::ComprisedCommand{drivers},
      fireCommand_{flywheel, drivers},
      feedCommand_{feeder, drivers},
      feederDelayTimer_{},
      feederIsFeeding_{}
{
    this->addSubsystemRequirement(flywheel);
    this->addSubsystemRequirement(feeder);
    this->comprisedCommandScheduler.registerSubsystem(flywheel);
    this->comprisedCommandScheduler.registerSubsystem(feeder);
}

void FireCommandGroup::initialize() {
    this->comprisedCommandScheduler.addCommand(&fireCommand_);
    feederDelayTimer_.restart(flywheel::FEEDER_DELAY_MS);
    feederIsFeeding_ = false;
}

void FireCommandGroup::execute()
{
    if ( feederIsFeeding_ == false && feederDelayTimer_.execute())
        {
            comprisedCommandScheduler.addCommand(&feedCommand_);
            feederIsFeeding_ = true;
        }
    this->comprisedCommandScheduler.run();
}

void FireCommandGroup::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&feedCommand_, interrupted);
    this->comprisedCommandScheduler.removeCommand(&fireCommand_, interrupted);
}

bool FireCommandGroup::isFinished() const 
{ 
    return false;
}

const char* FireCommandGroup::getName() const
{ 
    return "fire command group"; 
}

}  // namespace control



