#ifndef AUTO_AIM_POLICY_IMPL_HPP
#define AUTO_AIM_POLICY_IMPL_HPP

#include "auto_aim_policy.hpp"

namespace control::turret
{

template <typename Subsystem>
AutoAimPolicy<Subsystem>::AutoAimPolicy(Subsystem* const turret) :
    turret_{turret}
{
}

template <typename Subsystem>
AutoAimPolicy<Subsystem>::~AutoAimPolicy() = default;

template <typename Subsystem>
void AutoAimPolicy<Subsystem>::initialize()
{
    turret_->template initializeAiming<AimMode::Auto>();
}

template <typename Subsystem>
void AutoAimPolicy<Subsystem>::execute()
{
    turret_->template executeAiming<AimMode::Auto>();
}

template <typename Subsystem>
void AutoAimPolicy<Subsystem>::end(const bool)
{
    turret_->template stopAiming<AimMode::Auto>();
}

}

#endif // AUTO_AIM_POLICY_IMPL_HPP