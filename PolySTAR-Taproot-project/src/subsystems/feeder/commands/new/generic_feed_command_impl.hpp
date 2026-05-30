#ifndef GENERIC_FEED_COMMAND_IMPL_HPP
#define GENERIC_FEED_COMMAND_IMPL_HPP

#include "generic_feed_command.hpp"

namespace control::feeder
{

template <typename Subsytem, command_policy FeedPolicy>
GenericFeedCommand<Subsytem, FeedPolicy>::GenericFeedCommand(Subsytem* const feeder, src::Drivers* drivers)
    : tap::control::Command{},
        feeder_{feeder},
        drivers_{drivers},
        feedPolicy_{feeder}
{
    if (feeder == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

template <typename Subsytem, command_policy FeedPolicy>
GenericFeedCommand<Subsytem, FeedPolicy>::~GenericFeedCommand() = default;

template <typename Subsytem, command_policy FeedPolicy>
void GenericFeedCommand<Subsytem, FeedPolicy>::initialize()
{
    commandPolicy_.initialize();
}

template <typename Subsytem, command_policy FeedPolicy>
void GenericFeedCommand<Subsytem, FeedPolicy>::execute()
{
    commandPolicy_.execute();
}

template <typename Subsytem, command_policy FeedPolicy>
const char* GenericFeedCommand<Subsytem, FeedPolicy>::getName() const
{
    return NAME;
}

template <typename Subsytem, command_policy FeedPolicy>
bool GenericFeedCommand<Subsytem, FeedPolicy>::isFinished() const
{
    return false;
}

template <typename Subsytem, command_policy FeedPolicy>
void GenericFeedCommand<Subsytem, FeedPolicy>::end(const bool interrupt)
{
    commandPolicy_.end(interrupt);
}

} // namespace control::feeder

#endif // GENERIC_FEED_COMMAND_IMPL_HPP