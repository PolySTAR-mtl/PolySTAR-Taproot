#ifndef NORMAL_FIRE_POLICY_HPP
#define NORMAL_FIRE_POLICY_HPP

#include <cstdint>
#include <type_traits>

#include "subsystems/flywheel/core/flywheel_dji_subsystem.hpp"
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"

namespace control::flywheel
{

template <typename Subsystem>
class NormalFirePolicy
{
public:
    NormalFirePolicy(Subsystem* const flywheel, src::Drivers* drivers)
        : flywheel_{flywheel},
          drivers_{drivers},
          isKickstartDone_{false},
          startingTs_{}
    {
    }

    ~NormalFirePolicy() = default;

    void initialize()
    {
        if constexpr (std::is_same_v<Subsystem, control::flywheel::FlywheelDjiSubsystem>)
        {
            flywheel_->sendStartingBoost();

            isKickstartDone_ = false;
            startingTs_ = tap::arch::clock::getTimeMilliseconds();
        }
        else if constexpr (std::is_same_v<Subsystem, control::flywheel::FlywheelSubsystem>)
        {
            flywheel_->startFiring();
        }
    }

    void execute()
    {
        if constexpr (std::is_same_v<Subsystem, control::flywheel::FlywheelDjiSubsystem>)
        {
            if (!isKickstartDone_ &&
                tap::arch::clock::getTimeMilliseconds() - startingTs_ > KICKSTART_DELAY_MS)
            {
                flywheel_->startFiring();
                isKickstartDone_ = true;
            }
        }
    }

    void end(bool) { flywheel_->stopFiring(); }

private:
    Subsystem* const flywheel_;

    src::Drivers* drivers_;

    bool isKickstartDone_;
    uint32_t startingTs_;
};

}  // namespace control::flywheel

#endif