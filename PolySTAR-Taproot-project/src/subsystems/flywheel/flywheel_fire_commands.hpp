#ifndef FLYWHEEL_FIRE_COMMANDS_HPP
#define FLYWHEEL_FIRE_COMMANDS_HPP

#include "control/drivers/drivers.hpp"

#include "auto_fire_policy.hpp"
#include "normal_fire_policy.hpp"
#include "generic_fire_command.hpp"

namespace control::flywheel
{
using FlywheelAutoFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, AutoFirePolicy<FlywheelDjiSubsystem>>;

using FlywheelAutoFireCommand =
    GenericFireCommand<FlywheelSubsystem, AutoFirePolicy<FlywheelSubsystem>>;

using FlywheelFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, NormalFirePolicy<FlywheelDjiSubsystem>>;

using FlywheelFireCommand =
    GenericFireCommand<FlywheelSubsystem, NormalFirePolicy<FlywheelSubsystem>>;
}

#endif  