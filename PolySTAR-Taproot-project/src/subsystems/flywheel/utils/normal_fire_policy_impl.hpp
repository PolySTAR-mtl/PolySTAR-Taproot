#ifndef NORMAL_FIRE_POLICY_IMPL_HPP
#define NORMAL_FIRE_POLICY_IMPL_HPP

#include "normal_fire_policy.hpp"

namespace control::flywheel
{
template <typename Subsystem>
NormalFirePolicy<Subsystem>::NormalFirePolicy(Subsystem* const flywheel)
    : flywheel_{flywheel}
{
}

template <typename Subsystem>
NormalFirePolicy<Subsystem>::~NormalFirePolicy() = default;

template <typename Subsystem>
void NormalFirePolicy<Subsystem>::initialize()
{
    flywheel_->template initializeFiring<FireMode::Normal>();
}

template <typename Subsystem>
void NormalFirePolicy<Subsystem>::execute()
{
    flywheel_->template executeFiring<FireMode::Normal>();
}

template <typename Subsystem>
void NormalFirePolicy<Subsystem>::end(bool)
{ 
    flywheel_->stopFiring();
}

}

#endif // NORMAL_FIRE_POLICY_IMPL_HPP