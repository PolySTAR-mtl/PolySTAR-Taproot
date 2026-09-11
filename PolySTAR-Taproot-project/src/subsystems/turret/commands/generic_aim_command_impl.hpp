#ifndef GENERIC_AIM_COMMAND_IMPL_HPP
#define GENERIC_AIM_COMMAND_IMPL_HPP

#include "generic_aim_command.hpp"

namespace control::turret
{

template <typename Subsystem, command_policy AimPolicy>
GenericAimCommand<Subsystem, AimPolicy>::GenericAimCommand(Subsystem* const turret, src::Drivers* drivers) :
    tap::control::Command{},
    turret_{turret},
    drivers_{drivers},
    aimPolicy_{turret}
{
    if (turret == nullptr)
    {
        return;
    }

    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(turret));
}

template <typename Subsystem, command_policy AimPolicy>
GenericAimCommand<Subsystem, AimPolicy>::~GenericAimCommand() = default;


template <typename Subsystem, command_policy AimPolicy>
void GenericAimCommand<Subsystem, AimPolicy>::initialize()
{
    aimPolicy_.initialize();
}

template <typename Subsystem, command_policy AimPolicy>
void GenericAimCommand<Subsystem, AimPolicy>::execute()
{
    aimPolicy_.execute();
}

template <typename Subsystem, command_policy AimPolicy>
const char* GenericAimCommand<Subsystem, AimPolicy>::getName() const
{
    return NAME;
}

template <typename Subsystem, command_policy AimPolicy>
bool GenericAimCommand<Subsystem, AimPolicy>::isFinished() const
{
    return false;
}

template <typename Subsystem, command_policy AimPolicy>
void GenericAimCommand<Subsystem, AimPolicy>::end(const bool interrupt)
{
    aimPolicy_.end(interrupt);
}

} // namespace control::turret

#endif // GENERIC_AIM_COMMAND_IMPL_HPP