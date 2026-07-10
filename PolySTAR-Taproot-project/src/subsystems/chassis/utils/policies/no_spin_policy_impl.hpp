#ifndef NO_SPIN_POLICY_IMPL_HPP
#define NO_SPIN_POLICY_IMPL_HPP

#include "no_spin_policy.hpp"
#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <typename Subsystem>
NoSpinPolicy<Subsystem>::NoSpinPolicy(Subsystem* const chassis) :
    chassis_{chassis}
{
}

template <typename Subsystem>
NoSpinPolicy<Subsystem>::~NoSpinPolicy() = default;

template <typename Subsystem>
void NoSpinPolicy<Subsystem>::initialize()
{
    chassis_->template initializeSpinning<SpinMode::NoSpin>();
}

template <typename Subsystem>
void NoSpinPolicy<Subsystem>::execute()
{
    chassis_->template executeSpinning<SpinMode::NoSpin>();
}

template <typename Subsystem>
void NoSpinPolicy<Subsystem>::end(const bool)
{
    chassis_->template endSpinning<SpinMode::NoSpin>();
}

} // namespace control::chassis

#endif // NO_SPIN_POLICY_IMPL_HPP