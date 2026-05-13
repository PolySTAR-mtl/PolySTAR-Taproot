#ifndef FIRE_POLICY_HPP
#define FIRE_POLICY_HPP

#include <concepts>

namespace control::flywheel
{

template <typename Policy>
concept fire_policy =
    requires(Policy p, bool interrupt)
{
    { p.initialize() } -> std::same_as<void>;
    { p.execute() }        -> std::same_as<void>;
    { p.end(interrupt) }   -> std::same_as<void>;
};

} // namespace control::flywheel

#endif // FIRE_POLICY_HPP