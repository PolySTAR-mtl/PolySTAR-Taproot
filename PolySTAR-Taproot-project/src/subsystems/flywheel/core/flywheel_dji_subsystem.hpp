#ifndef FLYWHEEL_DJI_SUBSYSTEM_HPP
#define FLYWHEEL_DJI_SUBSYSTEM_HPP

#include <deque>

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "subsystems/flywheel/utils/snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "subsystems/flywheel/config/flywheel_constants.hpp"
#include "subsystems/sentry_general_constants.hpp"

namespace control::flywheel
{

/**
 * A bare bones Subsystem for interacting with a flywheel.
 */
class FlywheelDjiSubsystem : public FlywheelSubsystem
{
public:

    /**
     * Constructs a new FlywheelSubsystem with default parameters specified in
     * the private section of this class.
     */
    FlywheelDjiSubsystem(src::Drivers *drivers);

    FlywheelDjiSubsystem(const FlywheelDjiSubsystem &other) = delete;

    FlywheelDjiSubsystem &operator=(const FlywheelDjiSubsystem &other) = delete;

    ~FlywheelDjiSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void startFiring() override;

    void stopFiring() override;

    void sendStartingBoost();

    template <FireMode M>
    void initializeFiring() {}
    
    template <FireMode M>
    void executeFiring() {}

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_PWM_PIN = tap::gpio::Pwm::Pin::Z;

    src::Drivers* drivers_;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor leftMotor_;
    tap::motor::DjiMotor rightMotor_;

    float currentDjiSpeed_;

    bool isKickstartDone_;
    uint32_t startingTs_;
    tap::arch::MilliTimeout startMatchTimeout_;
    

};  // class FlywheelSubsystem

template <>
inline void FlywheelDjiSubsystem::initializeFiring<FireMode::AutoMode>() {
    isKickstartDone_ = false;
    startingTs_ = tap::arch::clock::getTimeMilliseconds();
    startMatchTimeout_.restart(START_MATCH_WAIT_TIME);
}

template <>
inline void FlywheelDjiSubsystem::executeFiring<FireMode::AutoMode>() {
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

template <>
inline void FlywheelDjiSubsystem::initializeFiring<FireMode::Normal>() {
    sendStartingBoost();
    isKickstartDone_ = false;
    startingTs_ = tap::arch::clock::getTimeMilliseconds();
}

template <>
inline void FlywheelDjiSubsystem::executeFiring<FireMode::Normal>() {
    if (!isKickstartDone_ &&
        tap::arch::clock::getTimeMilliseconds() - startingTs_ > KICKSTART_DELAY_MS)
    {
        startFiring();
        isKickstartDone_ = true;
    }
}

}  // namespace control::flywheel

#endif  // FLYWHEEL_SUBSYSTEM_HPP