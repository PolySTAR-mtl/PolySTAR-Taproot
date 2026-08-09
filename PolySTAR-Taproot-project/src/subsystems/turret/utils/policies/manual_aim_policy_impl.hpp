#ifndef MANUAL_AIM_POLICY_IMPL_HPP
#define MANUAL_AIM_POLICY_IMPL_HPP

#include "manual_aim_policy.hpp"
#include "subsystems/turret/utils/modes/aim_mode.hpp"
#include "subsystems/turret/utils/modes/spin_mode.hpp"

namespace control::turret
{

template <typename Subsystem, SpinMode SpinMode>
ManualAimPolicy<Subsystem, SpinMode>::ManualAimPolicy(Subsystem* const turret) :
    turret_{turret}
{
}

template <typename Subsystem, SpinMode SpinMode>
ManualAimPolicy<Subsystem, SpinMode>::~ManualAimPolicy() = default;

template <typename Subsystem, SpinMode SpinMode>
void ManualAimPolicy<Subsystem, SpinMode>::initialize()
{
    turret_->template initializeAiming<AimMode::Manual, SpinMode>();
}

template <typename Subsystem, SpinMode SpinMode>
void ManualAimPolicy<Subsystem, SpinMode>::execute()
{
    turret_->template executeAiming<AimMode::Manual, SpinMode>();
}

template <typename Subsystem, SpinMode SpinMode>
void ManualAimPolicy<Subsystem, SpinMode>::end(const bool)
{
    turret_->template stopAiming<AimMode::Manual, SpinMode>();
}

} //namespace control::turret

#endif // MANUAL_AIM_POLICY_IMPL_HPP