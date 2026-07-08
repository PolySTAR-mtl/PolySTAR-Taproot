#ifndef AUTO_AIM_POLICY_HPP
#define AUTO_AIM_POLICY_HPP

namespace control::turret
{

template <typename Subsystem>
class AutoAimPolicy
{
public:
    AutoAimPolicy(Subsystem* const turret);

    ~AutoAimPolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const turret_;
};

}

#include "auto_aim_policy_impl.hpp"

#endif //AUTO_AIM_POLICY_HPP