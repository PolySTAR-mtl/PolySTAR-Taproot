#ifndef FEEDER_REVERSE_COMMAND_HPP_
#define FEEDER_REVERSE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "feeder_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control {

namespace feeder {

class FeederReverseCommand : public tap::control::Command {

public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    FeederReverseCommand(FeederSubsystem *const turret, src::Drivers *drivers);

    FeederReverseCommand(const FeederReverseCommand &other) = delete;

    FeederReverseCommand &operator=(const FeederReverseCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder reverse command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    FeederSubsystem *const feeder;

    src::Drivers *drivers;
};  // FeederReverseCommand

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_REVERSE_COMMAND_HPP_
