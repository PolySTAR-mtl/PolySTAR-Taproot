#ifndef FEED_POLICY_HPP
#define FEED_POLICY_HPP

#include <concepts>

namespace control::feeder
{

template <typename Policy>
concept feed_policy =
    requires(Policy p, const bool interrupt)
{
    { p.initialize() } -> std::same_as<void>;
    { p.execute() }        -> std::same_as<void>;
    { p.end(interrupt) }   -> std::same_as<void>;
};

} // namespace control::feeder

#endif // FEED_POLICY_HPP