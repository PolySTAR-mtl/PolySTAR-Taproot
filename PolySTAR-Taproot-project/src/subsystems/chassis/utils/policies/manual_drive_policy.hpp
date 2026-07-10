#ifndef MANUAL_DRIVE_POLICY_HPP
#define MANUAL_DRIVE_POLICY_HPP

namespace control::chassis
{

template <typename Subsystem>
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