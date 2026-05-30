
#include "subsystems/feeder/core/feeder_velocity_subsystem.hpp";

namespace control::feeder
{
    template <FeedMode M>
    void FeederVelocitySubsystem::initializeFeed() {
        if constexpr (M == FeedMode::Auto) {
            startMatchTimeout.restart(START_MATCH_WAIT_TIME);
        } else if (M == FeedMode::Normal) {
            feeder->setDesiredOutput(FEEDER_RPM);
        }
    }

    template <FeedMode M>
    void FeederVelocitySubsystem::executeFeed() {
        if constexpr (M == FeedMode::Auto) {
            if (!startMatchTimeout.isExpired()) {
                feeder->setDesiredOutput(0);
            }
            drivers->leds.set(tap::gpio::Leds::B, true);
            bool shouldShoot = drivers->cvHandler.shouldShoot();
            if (shouldShoot) {
                feeder->setDesiredOutput(FEEDER_RPM);
            } else {
                feeder->setDesiredOutput(0);
            }
        } else if (M == FeedMode::Normal) {
            // Nothing for the moment
        }
    }
}