#ifndef FIRE_COMMAND_GROUP_HPP_
#define FIRE_COMMAND_GROUP_HPP_

#include "tap/control/comprised_command.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_subsystem.hpp"
#include "flywheel_fire_command.hpp"
#include "fire_end_command_group.hpp"
#include "subsystems/feeder/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/feeder_position_subsystem.hpp"

namespace control
{
class FireCommandGroup : public tap::control::ComprisedCommand
{
public:

    /**
     * Constructs a new Flywheel fire command
     * @param[in] flywheel a pointer to the flywheel to be passed in that this
     * @param[in] feeder a pointer to the feeder to be passed in that this
     * Command will interact with.
     */
    FireCommandGroup(flywheel::FlywheelSubsystem *const flywheel, feeder::FeederPositionSubsystem *const feeder, src::Drivers *drivers);

    FireCommandGroup(const FireCommandGroup &other) = delete;

    FireCommandGroup &operator=(const FireCommandGroup &other) = delete;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char *getName() const override { return "fire command group"; }

private:
    // attributes needed to operate the group command 
    flywheel::FlywheelFireCommand fireCommand;

    feeder::FeederMoveUnjamCommand feedCommand;

    src::Drivers *drivers;

    tap::arch::MilliTimeout feederDelayTimer;

    bool feederIsFeeding;

};  // class FireCommandGroup

}  // namespace control

#endif  // FIRE_COMMAND_GROUP_HPP_
