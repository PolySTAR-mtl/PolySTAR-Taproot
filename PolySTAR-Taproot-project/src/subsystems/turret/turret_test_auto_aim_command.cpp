#include "turret_test_auto_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace turret
{
TurretTestAutoAimCommand::TurretTestAutoAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : GenericAutoAimCommand(turret, drivers)
{
}

void  TurretTestAutoAimCommand::execute()
{
    GenericAutoAimCommand::execute();
}
}  // namespace turret
}  // namespace control

