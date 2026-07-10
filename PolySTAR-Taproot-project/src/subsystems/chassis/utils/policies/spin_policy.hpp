#ifndef SPIN_POLICY_HPP
#define SPIN_POLICY_HPP

namespace control::chassis
{

template <typename Subsystem>
class SpinPolicy
{
public:

    SpinPolicy(Subsystem* const chassis);

    ~SpinPolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const chassis_;
};

} // namespace control::chassis

#include "spin_policy_impl.hpp"

#endif // SPIN_POLICY_HPP