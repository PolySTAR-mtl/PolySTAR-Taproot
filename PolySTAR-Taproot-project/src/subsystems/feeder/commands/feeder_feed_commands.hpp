#ifndef FEEDER_FEED_COMMANDS_HPP
#define FEEDER_FEED_COMMANDS_HPP

#include "control/drivers/drivers.hpp"


#include "subsystems/feeder/utils/auto_feed_policy.hpp"
#include "subsystems/feeder/utils/normal_feed_policy.hpp"
#include "subsystems/feeder/commands/generic_feed_command.hpp"

namespace control::feeder 
{
    using FeederFeedCommand = 
        GenericFeedCommand<FeederVelocitySubsystem, NormalFeedPolicy<FeederVelocitySubsystem>>;

    using AutoFeedCommand = 
        GenericFeedCommand<FeederVelocitySubsystem, AutoFeedPolicy<FeederVelocitySubsystem>>;
}

#endif // FEEDER_FEED_COMMANDS_HPP