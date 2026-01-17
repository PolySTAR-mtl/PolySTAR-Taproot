#ifndef FLYWHEEL_FIRE_COMMAND_HPP_
#define FLYWHEEL_FIRE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "flywheel_subsystem.hpp"
#include "generic_fire_command.hpp"
#include "normal_fire_policy.hpp"

namespace control
{
namespace flywheel
{
using FlywheelFireCommand =
    GenericFireCommand<FlywheelSubsystem, NormalFirePolicy<FlywheelSubsystem>>;

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_FIRE_COMMAND_HPP
