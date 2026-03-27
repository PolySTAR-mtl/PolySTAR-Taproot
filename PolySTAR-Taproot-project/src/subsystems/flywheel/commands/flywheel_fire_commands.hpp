#ifndef FLYWHEEL_FIRE_COMMANDS_HPP
#define FLYWHEEL_FIRE_COMMANDS_HPP

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/utils/auto_fire_policy.hpp"
#include "subsystems/flywheel/utils/normal_fire_policy.hpp"
#include "subsystems/flywheel/commands/generic_fire_command.hpp"

namespace control::flywheel
{
using FlywheelAutoFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, AutoFirePolicy<FlywheelDjiSubsystem>>;

using FlywheelFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, NormalFirePolicy<FlywheelDjiSubsystem>>;

using FlywheelFireCommand =
    GenericFireCommand<FlywheelSubsystem, NormalFirePolicy<FlywheelSubsystem>>;
}

#endif  