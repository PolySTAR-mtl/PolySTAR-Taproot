#include "subsystems/feeder/core/feeder_velocity_subsystem.hpp"
#include "subsystems/feeder/config/feeder_config.hpp"
#include "subsystems/sentry_general_constants.hpp"

namespace control::feeder
{
    template <FeedMode M>
    void FeederVelocitySubsystem::initializeFeed() {
        if constexpr (M == FeedMode::Auto) {
            startMatchTimeout.restart(START_MATCH_WAIT_TIME);
        } else if constexpr (M == FeedMode::Normal) {
            setDesiredOutput(ACTIVE_FEEDER_CONFIG.feederRpm);
        }
    }

    template <FeedMode M>
    void FeederVelocitySubsystem::executeFeed() {
        if constexpr (M == FeedMode::Auto) {
            if (!startMatchTimeout.isExpired()) {
                setDesiredOutput(0);
            }

            srcDrivers->leds.set(tap::gpio::Leds::B, true);

            bool shouldShoot = srcDrivers->cvHandler.shouldShoot();
            if (shouldShoot) {
                setDesiredOutput(ACTIVE_FEEDER_CONFIG.feederRpm);
            } else {
                setDesiredOutput(0);
            }
        } else if constexpr (M == FeedMode::Normal) {
            // Nothing for the moment
        }
    }
} // namespace control::feeder