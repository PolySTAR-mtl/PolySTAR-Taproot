#ifndef GENERIC_FEED_COMMAND_HPP
#define GENERIC_FEED_COMMAND_HPP

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "subsystems/feeder/core/feeder_velocity_subsystem.hpp"
#include "subsystems/feeder/concepts/feed_policy.hpp"

namespace control::feeder
{

template <typename Subsytem, feed_policy FeedPolicy>
class GenericFeedCommand : public tap::control::Command
{
public:
    GenericFeedCommand(Subsytem* const feeder, src::Drivers* drivers);

    ~GenericFeedCommand();

    GenericFeedCommand(const GenericFeedCommand& other) = delete;

    GenericFeedCommand& operator=(const GenericFeedCommand& other) = delete;

    void initialize() override;

    void execute() override;

    const char* getName() const override;

    bool isFinished() const override;

    void end(const bool interrupt) override;
private:
    static constexpr const char* NAME = "feeder feed command";
    Subsytem* const feeder_;
    src::Drivers* drivers_;
    FeedPolicy feedPolicy_;
};

} // namespace control::feeder

#include "generic_feed_command_impl.hpp"

#endif // GENERIC_FEED_COMMAND_HPP