#ifndef SPIN_POLICY_IMPL_HPP
#define SPIN_POLICY_IMPL_HPP

#include "spin_policy.hpp"
#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <typename Subsystem>
SpinPolicy<Subsystem>::SpinPolicy(Subsystem* const chassis) :
    chassis_{chassis}
{
}

template <typename Subsystem>
SpinPolicy<Subsystem>::~SpinPolicy() = default;

template <typename Subsystem>
void SpinPolicy<Subsystem>::initialize()
{
    chassis_->template initializeSpinning<SpinMode::Spin>();
}

template <typename Subsystem>
void SpinPolicy<Subsystem>::execute()
{
    chassis_->template executeSpinning<SpinMode::Spin>();
}

template <typename Subsystem>
void SpinPolicy<Subsystem>::end(const bool)
{
    chassis_->template endSpinning<SpinMode::Spin>();
}

} // namespace control::chassis

#endif // SPIN_POLICY_IMPL_HPP