#ifndef AUTO_FIRE_POLICY_HPP
#define AUTO_FIRE_POLICY_HPP

#include <cstdint>
#include <type_traits>

#include "subsystems/flywheel/core/flywheel_dji_subsystem.hpp"

namespace control::flywheel
{

template <typename Subsystem>
class AutoFirePolicy
{
public:
    AutoFirePolicy(Subsystem* const flywheel);

    ~AutoFirePolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const flywheel_;
};

}  // namespace control::flywheel

#include "auto_fire_policy_impl.hpp"

#endif