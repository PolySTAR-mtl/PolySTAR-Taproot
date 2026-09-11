#ifndef FLYWHEEL_DJI_SUBSYSTEM_IMPL_HPP
#define FLYWHEEL_DJI_SUBSYSTEM_IMPL_HPP

#include "flywheel_dji_subsystem.hpp"

namespace control::flywheel
{

template <FireMode M>
inline void FlywheelDjiSubsystem::initializeFiring() {
    startingTs_ = tap::arch::clock::getTimeMilliseconds();
    if constexpr (M == FireMode::Auto) {
        startMatchTimeout_.restart(START_MATCH_WAIT_TIME);
    }
    else if constexpr (M == FireMode::Normal) {
        sendStartingBoost();
    }
}

template <FireMode M>
inline void FlywheelDjiSubsystem::executeFiring() {
    
    //FlywheelState state = getState();
    if constexpr (M == FireMode::Auto) {
        /*
        if (!startMatchTimeout_.isExpired())
        {
            stopFiring();
            return;
        }

        drivers_->leds.set(tap::gpio::Leds::C, true);

        if (!drivers_->cvHandler.shouldShoot())
        {
            stopFiring();
            return;
        }

        const uint32_t currentTs = tap::arch::clock::getTimeMilliseconds();
        /// TODO: Fix a possible bug on next line.
        if (!currentTs - startingTs_ < KICKSTART_DELAY_MS)
        {
            sendStartingBoost();
        }
        else if (state == FlywheelState::Starting)
        {
            startFiring();
            isKickstartDone_ = true;
        }
        */
    }
    else if constexpr (M == FireMode::Normal) {
        if (tap::arch::clock::getTimeMilliseconds() - startingTs_ > KICKSTART_DELAY_MS)
        {
            startFiring();
        }
    }
}

} // namespace control::flywheel

#endif // FLYWHEEL_DJI_SUBSYSTEM_IMPL_HPP