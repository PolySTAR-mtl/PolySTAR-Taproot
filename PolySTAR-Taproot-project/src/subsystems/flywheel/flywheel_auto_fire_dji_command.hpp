#pragma once

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "auto_fire_policy.hpp"
#include "flywheel_dji_subsystem.hpp"
#include "generic_fire_command.hpp"

namespace control
{
namespace flywheel
{

using FlywheelAutoFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, AutoFirePolicy<FlywheelDjiSubsystem>>;

}  // namespace flywheel

}  // namespace control
