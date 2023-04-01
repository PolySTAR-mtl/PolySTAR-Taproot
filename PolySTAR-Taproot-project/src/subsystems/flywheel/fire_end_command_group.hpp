#ifndef FIRE_END_COMMAND_GROUP_HPP_
#define FIRE_END_COMMAND_GROUP_HPP_

#include "tap/control/comprised_command.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_subsystem.hpp"
#include "flywheel_fire_command.hpp"
#include "subsystems/feeder/feeder_reverse_command.hpp"

namespace control
{
namespace flywheel
{
/**
 * 
 */
class FireEndCommandGroup : public tap::control::ComprisedCommand
{
public:

    /**
     * Constructs a new Flywheel fire command
     * @param[in] flywheel a pointer to the flywheel to be passed in that this
     * @param[in] feeder a pointer to the feeder to be passed in that this
     * Command will interact with.
     */
    FireEndCommandGroup(FlywheelSubsystem *const flywheel, feeder::FeederSubsystem *const feeder, src::Drivers *drivers);

    FireEndCommandGroup(const FireEndCommandGroup &other) = delete;

    FireEndCommandGroup &operator=(const FireEndCommandGroup &other) = delete;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char *getName() const override { return "fire end command group"; }

private:
    // attributes needed to operate the group command 
    FlywheelFireCommand fireCommand;

    feeder::FeederReverseCommand reverseFeedCommand;

    src::Drivers *drivers;

    tap::arch::MilliTimeout flywheelDelayTimer;

};  // class FireEndCommandGroup

}  // namespace flywheel

}  // namespace control

#endif  // FIRE_END_COMMAND_GROUP_HPP_
