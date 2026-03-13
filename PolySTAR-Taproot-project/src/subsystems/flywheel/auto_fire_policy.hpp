#ifndef AUTO_FIRE_POLICY_HPP
#define AUTO_FIRE_POLICY_HPP

#include <cstdint>
#include <type_traits>

#include "flywheel_dji_subsystem.hpp"

namespace control::flywheel
{

template <typename Subsystem>
class AutoFirePolicy
{
public:
    AutoFirePolicy(Subsystem* const flywheel, src::Drivers* drivers)
        : flywheel_{flywheel},
          drivers_{drivers},
          isKickstartDone_{false},
          startingTs_{},
          startMatchTimeout_{}
    {
    }

    ~AutoFirePolicy() = default;

    void initialize()
    {
        if constexpr (std::is_same<Subsystem, FlywheelDjiSubsystem>)
        {
            isKickstartDone_ = false;
            startingTs_ = tap::arch::clock::getTimeMilliseconds();

            startMatchTimeout_.restart(START_MATCH_WAIT_TIME);
            return;
        }
    }

    void execute()
    {
        if constexpr (std::is_same<Subsystem, FlywheelDjiSubsystem>)
        {
            if (!startMatchTimeout_.isExpired())
            {
                flywheel_->stopFiring();
                isKickstartDone_ = false;
                return;
            }

            drivers_->leds.set(tap::gpio::Leds::C, true);

            if (!drivers_->cvHandler.shouldShoot())
            {
                flywheel_->stopFiring();
                isKickstartDone_ = false;
                return;
            }

            uint32_t currentTs = tap::arch::clock::getTimeMilliseconds();
            /// TODO: Fix a bug on next line.
            if (!currentTs - startingTs_ < KICKSTART_DELAY_MS)
            {
                flywheel_->sendStartingBoost();
            }
            else if (!isKickstartDone_)
            {
                flywheel_->startFiring();  // will send default speeds to DjiMotors
                isKickstartDone_ = true;
            }
            return;
        }
    }

    void end(bool) { flywheel_->stopFiring(); }

private:
    Subsystem* const flywheel_;
    src::Drivers* drivers_;

    bool isKickstartDone_;
    uint32_t startingTs_;

    tap::arch::MilliTimeout startMatchTimeout_;
};

}  // namespace control::flywheel

#endif