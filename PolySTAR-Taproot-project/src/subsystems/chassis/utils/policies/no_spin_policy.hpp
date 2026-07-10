#ifndef NO_SPIN_POLICY_HPP
#define NO_SPIN_POLICY_HPP

namespace control::chassis
{

template <typename Subsystem>
class NoSpinPolicy
{
public:

    NoSpinPolicy(Subsystem* const chassis);

    ~NoSpinPolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const chassis_;
};

} // namespace control::chassis

#include "no_spin_policy_impl.hpp"

#endif // NO_SPIN_POLICY_HPP