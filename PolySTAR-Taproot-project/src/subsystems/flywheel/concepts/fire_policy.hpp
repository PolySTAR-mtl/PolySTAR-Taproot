#ifndef FIRE_POLICY_HPP
#define FIRE_POLICY_HPP

#include <concepts>

#include "control/drivers/drivers.hpp"

namespace control::flywheel
{

template <typename Policy, typename Subsystem>
concept fire_policy =
    requires(Policy policy, Subsystem* subsystem, src::Drivers* drivers, bool interrupt)
{
    { Policy{subsystem, drivers} };

    { policy.initialize() } -> std::same_as<void>;
    { policy.execute() } -> std::same_as<void>;
    { policy.end(interrupt) } -> std::same_as<void>;
};

}


#endif  // FIRE_POLICY_HPP