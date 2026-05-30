#ifndef AUTO_FIRE_POLICY_IMPL_HPP
#define AUTO_FIRE_POLICY_IMPL_HPP

#include "auto_fire_policy.hpp"

namespace control::flywheel
{

template <typename Subsystem>
AutoFirePolicy<Subsystem>::AutoFirePolicy(Subsystem* const flywheel)
    : flywheel_{flywheel}
{
}

template <typename Subsystem>
AutoFirePolicy<Subsystem>::~AutoFirePolicy() = default;

template <typename Subsystem>
void AutoFirePolicy<Subsystem>::initialize()
{
    flywheel_->template initializeFiring<FireMode::Auto>();
}

template <typename Subsystem>
void AutoFirePolicy<Subsystem>::execute()
{
    flywheel_->template executeFiring<FireMode::Auto>();
}

template <typename Subsystem>
void AutoFirePolicy<Subsystem>::end(bool) { flywheel_->stopFiring(); }

} // namespace control::flywheel

#endif // AUTO_FIRE_POLICY_IMPL_HPP