#ifndef MANUAL_AIM_POLICY_IMPL_HPP
#define MANUAL_AIM_POLICY_IMPL_HPP

#include "manual_aim_policy.hpp"
#include "aim_mode.hpp"

namespace control::turret
{

template <typename Subsystem>
ManualAimPolicy<Subsystem>::ManualAimPolicy(Subsystem* const turret) :
    turret_{turret}
{
}

template <typename Subsystem>
ManualAimPolicy<Subsystem>::~ManualAimPolicy() = default;

template <typename Subsystem>
void ManualAimPolicy<Subsystem>::initialize()
{
    turret_->template initializeAiming<AimMode::Manual>();
}

template <typename Subsystem>
void ManualAimPolicy<Subsystem>::execute()
{
    turret_->template executeAiming<AimMode::Manual>();
}

template <typename Subsystem>
void ManualAimPolicy<Subsystem>::end(const bool)
{
    turret_->template stopAiming<AimMode::Manual>();
}

} //namespace control::turret

#endif // MANUAL_AIM_POLICY_IMPL_HPP