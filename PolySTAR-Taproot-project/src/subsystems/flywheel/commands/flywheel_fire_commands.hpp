#ifndef FLYWHEEL_FIRE_COMMANDS_HPP
#define FLYWHEEL_FIRE_COMMANDS_HPP

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/utils/auto_fire_policy.hpp"
#include "subsystems/flywheel/utils/normal_fire_policy.hpp"
#include "subsystems/flywheel/commands/generic_fire_command.hpp"

namespace control::flywheel
{
using AutoFireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, AutoFirePolicy<FlywheelDjiSubsystem>>;

using AutoFireCommand =
    GenericFireCommand<FlywheelSubsystem, AutoFirePolicy<FlywheelSubsystem>>;

using FireDjiCommand =
    GenericFireCommand<FlywheelDjiSubsystem, NormalFirePolicy<FlywheelDjiSubsystem>>;

using FireCommand =
    GenericFireCommand<FlywheelSubsystem, NormalFirePolicy<FlywheelSubsystem>>;
}

#endif // FLYWHEEL_FIRE_COMMANDS_HPP