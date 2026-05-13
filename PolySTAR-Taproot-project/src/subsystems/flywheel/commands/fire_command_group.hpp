#ifndef FIRE_COMMAND_GROUP_HPP_
#define FIRE_COMMAND_GROUP_HPP_

#include "tap/control/comprised_command.hpp"
#include "control/drivers/drivers.hpp"
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "subsystems/flywheel/commands/flywheel_fire_commands.hpp"
#include "subsystems/flywheel/commands/fire_end_command_group.hpp"
#include "subsystems/feeder/commands/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/core/feeder_position_subsystem.hpp"

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

    bool isFinished() const override;

    const char* getName() const override;

private:
    // attributes needed to operate the group command
    flywheel::FlywheelFireCommand fireCommand_;

    feeder::FeederMoveUnjamCommand feedCommand_;

    tap::arch::MilliTimeout feederDelayTimer_;

    bool feederIsFeeding_;
};  // class FireCommandGroup

}  // namespace control

#endif  // FIRE_COMMAND_GROUP_HPP_
