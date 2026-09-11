#ifndef MANUAL_AIM_POLICY_HPP
#define MANUAL_AIM_POLICY_HPP

#include "subsystems/turret/utils/modes/spin_mode.hpp"

namespace control::turret
{

template <typename Subsystem, SpinMode spinMode>
class ManualAimPolicy
{
public:

    ManualAimPolicy(Subsystem* const turret);

    ~ManualAimPolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const turret_;
};

} // namespace control::turret

#include "manual_aim_policy_impl.hpp"

#endif // MANUAL_AIM_POLICY_HPP