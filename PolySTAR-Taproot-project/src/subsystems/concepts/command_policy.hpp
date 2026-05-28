#ifndef COMMAND_POLICY_HPP_
#define COMMAND_POLICY_HPP_

#include <concepts>


namespace control
{
template <typename Policy>
concept command_policy =
    requires(Policy p, const bool interrupt)
{
    { p.initialize() } -> std::same_as<void>;
    { p.execute() }        -> std::same_as<void>;
    { p.end(interrupt) }   -> std::same_as<void>;
};
}

#endif