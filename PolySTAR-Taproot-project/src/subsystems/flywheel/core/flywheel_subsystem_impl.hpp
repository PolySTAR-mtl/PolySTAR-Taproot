#ifndef FLYWHEEL_SUBSYSTEM_IMPL_HPP
#define FLYWHEEL_SUBSYSTEM_IMPL_HPP

#include "flywheel_subsystem.hpp"

namespace control::flywheel
{
    template <FireMode M>
    void FlywheelSubsystem::initializeFiring()
    {
        if constexpr (M == FireMode::Normal) {
            startFiring();
        }
    }

    template <FireMode M>
    void FlywheelSubsystem::executeFiring()
    {}
}

#endif // FLYWHEEL_SUBSYSTEM_IMPL_HPP