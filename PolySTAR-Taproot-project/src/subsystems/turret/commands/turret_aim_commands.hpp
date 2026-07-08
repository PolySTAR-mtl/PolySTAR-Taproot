#ifndef TURRET_AIM_COMMANDS_HPP
#define TURRET_AIM_COMMANDS_HPP

#include "generic_aim_command.hpp"
#include "subsystems/turret/utils/auto_aim_policy.hpp"
#include "subsystems/turret/utils/manual_aim_policy.hpp"
#include "subsystems/turret/core/turret_subsystem.hpp"

namespace control::turret 
{
    using ManualAimCommand = 
        GenericAimCommand<TurretSubsystem, ManualAimPolicy<TurretSubsystem>>;

    using AutoAimCommand = 
        GenericAimCommand<TurretSubsystem, AutoAimPolicy<TurretSubsystem>>;

}

#endif // TURRET_AIM_COMMANDS_HPP