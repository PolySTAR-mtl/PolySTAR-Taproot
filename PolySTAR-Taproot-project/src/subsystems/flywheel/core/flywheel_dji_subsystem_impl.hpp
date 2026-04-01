#ifndef FLYWHEEL_SUBSYSTEM_IMPL_HPP
#define FLYWHEEL_SUBSYSTEM_IMPL_HPP

#include "flywheel_dji_subsystem.hpp"

namespace control::flywheel
{
 
template <FireMode M>
inline void FlywheelDjiSubsystem::initializeFiring() {
    if constexpr (M == FireMode::AutoMode) {
        isKickstartDone_ = false;
        startingTs_ = tap::arch::clock::getTimeMilliseconds();
        startMatchTimeout_.restart(START_MATCH_WAIT_TIME);
    }
    else if constexpr (M == FireMode::Normal) {
        sendStartingBoost();
        isKickstartDone_ = false;
        startingTs_ = tap::arch::clock::getTimeMilliseconds();
    }
}

template <FireMode M>
inline void FlywheelDjiSubsystem::executeFiring() {
    if constexpr (M == FireMode::AutoMode) {
        if (!startMatchTimeout_.isExpired())
        {
            stopFiring();
            isKickstartDone_ = false;
            return;
        }

        drivers_->leds.set(tap::gpio::Leds::C, true);

        if (!drivers_->cvHandler.shouldShoot())
        {
            stopFiring();
            isKickstartDone_ = false;
            return;
        }

        const uint32_t currentTs = tap::arch::clock::getTimeMilliseconds();
        /// TODO: Fix a possible bug on next line.
        if (!currentTs - startingTs_ < KICKSTART_DELAY_MS)
        {
            sendStartingBoost();
        }
        else if (!isKickstartDone_)
        {
            startFiring();
            isKickstartDone_ = true;
        }
    }
    else if constexpr (M == FireMode::Normal) {
        if (!isKickstartDone_ &&
            tap::arch::clock::getTimeMilliseconds() - startingTs_ > KICKSTART_DELAY_MS)
        {
            startFiring();
            isKickstartDone_ = true;
        }
    }
}

}

#endif // FLYWHEEL_SUBSYSTEM_IMPL_HPP