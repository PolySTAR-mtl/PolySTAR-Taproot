#ifndef AUTO_DRIVE_POLICY_HPP
#define AUTO_DRIVE_POLICY_HPP

#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <typename Subsystem, SpinMode spinMode>
class AutoDrivePolicy
{
public:
    AutoDrivePolicy(Subsystem* const chassis);

    ~AutoDrivePolicy();

    void initialize();

    void execute();

    void end(const bool interrupt);

private:
    Subsystem* const chassis_;
};

} // namespace control::chassis

#include "auto_drive_policy_impl.hpp"

#endif // AUTO_DRIVE_POLICY_HPP