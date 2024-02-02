#ifndef TURRET_TEST_AUTO_AIM_COMMAND_HPP_
#define TURRET_TEST_AUTO_AIM_COMMAND_HPP_

#include "generic_auto_aim_command.hpp"

namespace control
{
namespace turret
{
class TurretTestAutoAimCommand : public GenericAutoAimCommand
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    TurretTestAutoAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretTestAutoAimCommand(const TurretTestAutoAimCommand &other) = delete;

    TurretTestAutoAimCommand &operator=(const TurretTestAutoAimCommand &other) = delete;

    const char *getName() const { return "turret test auto aim command"; }

    void execute() override;

};  // TurretTestAutoAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_TEST_AUTO_AIM_COMMAND_HPP_

