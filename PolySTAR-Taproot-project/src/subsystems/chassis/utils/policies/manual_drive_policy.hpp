#ifndef MANUAL_DRIVE_POLICY_HPP
#define MANUAL_DRIVE_POLICY_HPP

#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <typename Subsystem, SpinMode spinMode>
class ManualDrivePolicy
{
public:
    ManualDrivePolicy(Subsystem* const chassis);

    ~ManualDrivePolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const chassis_;
};

} // namespace control::chassis

#include "manual_drive_policy_impl.hpp"

#endif // MANUAL_DRIVE_POLICY_HPP