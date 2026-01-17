#ifndef FLYWHEEL_FIRE_DJI_COMMAND_HPP_
#define FLYWHEEL_FIRE_DJI_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "auto_fire_policy.hpp"
#include "flywheel_dji_subsystem.hpp"
#include "generic_fire_command.hpp"
#include "normal_fire_policy.hpp"

namespace control
{
namespace flywheel
{

using FlywheelFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, NormalFirePolicy<FlywheelDjiSubsystem>>;

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_FIRE_COMMAND_HPP
