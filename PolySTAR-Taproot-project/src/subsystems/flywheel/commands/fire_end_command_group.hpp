#ifndef FIRE_END_COMMAND_GROUP_HPP_
#define FIRE_END_COMMAND_GROUP_HPP_

#include "tap/control/comprised_command.hpp"
#include "control/drivers/drivers.hpp"
#include "subsystems/feeder/feeder_feed_command.hpp"
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "flywheel_fire_commands.hpp"
#include "subsystems/feeder/feeder_position_subsystem.hpp"

namespace control
{
class FireEndCommandGroup : public tap::control::ComprisedCommand
{
public:

    /**
     * Constructs a new Flywheel fire command
     * @param[in] flywheel a pointer to the flywheel to be passed in that this
     * @param[in] feeder a pointer to the feeder to be passed in that this
     * Command will interact with.
     */
    FireEndCommandGroup(flywheel::FlywheelSubsystem *const flywheel, feeder::FeederPositionSubsystem *const feeder, src::Drivers *drivers);

    FireEndCommandGroup(const FireEndCommandGroup &other) = delete;

    FireEndCommandGroup &operator=(const FireEndCommandGroup &other) = delete;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override;

private:
    // attributes needed to operate the group command 
    flywheel::FlywheelFireCommand fireCommand;
    
    tap::arch::MilliTimeout flywheelDelayTimer;
};  // class FireEndCommandGroup

}  // namespace control

#endif  // FIRE_END_COMMAND_GROUP_HPP_
