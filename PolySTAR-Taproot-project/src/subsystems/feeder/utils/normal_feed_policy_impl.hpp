#ifndef NORMAL_FEED_POLICY_IMPL_HPP
#define NORMAL_FEED_POLICY_IMPL_HPP

#include "subsystems/feeder/utils/normal_feed_policy.hpp"
#include "subsystems/feeder/utils/feed_mode.hpp"

namespace control::feeder
{
template<typename Subsystem>
NormalFeedPolicy<Subsystem>::NormalFeedPolicy(Subsystem* const feeder)
    : feeder_{feeder} 
{
}

template<typename Subsystem>
NormalFeedPolicy<Subsystem>::~NormalFeedPolicy() = default;

template<typename Subsystem>
void NormalFeedPolicy<Subsystem>::initialize() 
{
    feeder_->template initializeFeed<FeedMode::Normal>();
}

template<typename Subsystem>
void NormalFeedPolicy<Subsystem>::execute()
{
    feeder_->template executeFeed<FeedMode::Normal>();
}

template<typename Subsystem>
void NormalFeedPolicy<Subsystem>::end(bool interrupt) 
{
    feeder_->template stopFiring();
}
}

#endif // NORMAL_FEED_POLICY_IMPL_HPP