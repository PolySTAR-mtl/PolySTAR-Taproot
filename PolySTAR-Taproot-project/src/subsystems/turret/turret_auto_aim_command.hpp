#ifndef TURRET_AUTO_AIM_COMMAND_HPP_
#define TURRET_AUTO_AIM_COMMAND_HPP_

#include "generic_auto_aim_command.hpp"

namespace control
{
namespace turret
{
class TurretAutoAimCommand : public GenericAutoAimCommand
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    TurretAutoAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretAutoAimCommand(const TurretAutoAimCommand &other) = delete;

    TurretAutoAimCommand &operator=(const TurretAutoAimCommand &other) = delete;

    const char *getName() const { return "turret auto aim command"; }

    void execute() override;

};  // TurretAutoAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_AUTO_AIM_COMMAND_HPP_

