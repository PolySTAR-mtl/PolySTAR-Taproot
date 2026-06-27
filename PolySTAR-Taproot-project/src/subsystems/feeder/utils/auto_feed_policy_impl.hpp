#ifndef AUTO_FEED_POLICY_IMPL_HPP
#define AUTO_FEED_POLICY_IMPL_HPP

#include "subsystems/feeder/utils/auto_feed_policy.hpp"
#include "subsystems/feeder/utils/feed_mode.hpp"

namespace control::feeder
{

template<typename Subsystem>
AutoFeedPolicy<Subsystem>::AutoFeedPolicy(Subsystem* const feeder)
    : feeder_{feeder} 
{
}

template<typename Subsystem>
AutoFeedPolicy<Subsystem>::~AutoFeedPolicy() = default;

template<typename Subsystem>
void AutoFeedPolicy<Subsystem>::initialize()
{
    feeder_->template initializeFeed<FeedMode::Auto>();
}

template<typename Subsystem>
void AutoFeedPolicy<Subsystem>::execute() 
{
    feeder_->template executeFeed<FeedMode::Auto>();
}

template<typename Subsystem>
void AutoFeedPolicy<Subsystem>::end(bool interrupt) {}

} // control::feeder

#endif //AUTO_FEED_POLICY_IMPL_HPP