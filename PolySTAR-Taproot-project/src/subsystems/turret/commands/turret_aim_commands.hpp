#ifndef TURRET_AIM_COMMANDS_HPP
#define TURRET_AIM_COMMANDS_HPP

#include "subsystems/turret/commands/generic_aim_command.hpp"
#include "subsystems/turret/utils/policies/auto_aim_policy.hpp"
#include "subsystems/turret/utils/policies/manual_aim_policy.hpp"
#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/utils/modes/spin_mode.hpp"

namespace control::turret
{
    using ManualAimCommand =
        GenericAimCommand<TurretSubsystem, ManualAimPolicy<TurretSubsystem, SpinMode::NoSpin>>;

    using ManualSpinAimCommand =
        GenericAimCommand<TurretSubsystem, ManualAimPolicy<TurretSubsystem, SpinMode::Spin>>;

    using AutoAimCommand =
        GenericAimCommand<TurretSubsystem, AutoAimPolicy<TurretSubsystem>>;

}

#endif // TURRET_AIM_COMMANDS_HPP