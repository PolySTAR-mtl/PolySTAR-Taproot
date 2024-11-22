#ifndef GENERIC_AUTO_AIM_COMMAND_HPP_
#define GENERIC_AUTO_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class GenericAutoAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    GenericAutoAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    GenericAutoAimCommand(const GenericAutoAimCommand &other) = delete;

    GenericAutoAimCommand &operator=(const GenericAutoAimCommand &other) = delete;

    void initialize() override;

    virtual void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    TurretSubsystem *const turret;

    src::Drivers *drivers;

    static constexpr float MRAD_TO_DEGREES = 0.0572958;

};  // GenericAutoAimCommand

}  // namespace turret

}  // namespace control

#endif  // GENERIC_AUTO_AIM_COMMAND_HPP_
