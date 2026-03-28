#ifndef NORMAL_FIRE_POLICY_HPP
#define NORMAL_FIRE_POLICY_HPP

#include <cstdint>
#include <type_traits>

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/core/flywheel_dji_subsystem.hpp"
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"

namespace control::flywheel
{

template <typename Subsystem>
class NormalFirePolicy
{
public:
    NormalFirePolicy(Subsystem* const flywheel);

    ~NormalFirePolicy();

    void initialize();

    void execute();

    void end(bool interrupt);

private:
    Subsystem* const flywheel_;

    src::Drivers* drivers_;

    bool isKickstartDone_;
    uint32_t startingTs_;
};

}  // namespace control::flywheel

#include "normal_fire_policy_impl.hpp"

#endif