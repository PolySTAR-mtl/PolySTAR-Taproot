#ifndef MANUAL_DRIVE_POLICY_IMPL_HPP
#define MANUAL_DRIVE_POLICY_IMPL_HPP

#include "manual_drive_policy.hpp"
#include "subsystems/chassis/utils/modes/drive_mode.hpp"

namespace control::chassis
{

template <typename Subsystem>
ManualDrivePolicy<Subsystem>::ManualDrivePolicy(Subsystem* const chassis) :
    chassis_{chassis}
{
}

template <typename Subsystem>
ManualDrivePolicy<Subsystem>::~ManualDrivePolicy() = default;

template <typename Subsystem>
void ManualDrivePolicy<Subsystem>::initialize()
{
    chassis_->template initializeDriving<DriveMode::Manual>();
}

template <typename Subsystem>
void ManualDrivePolicy<Subsystem>::execute()
{
    chassis_->template executeDriving<DriveMode::Manual>();
}

template <typename Subsystem>
void ManualDrivePolicy<Subsystem>::end(const bool)
{
    chassis_->template endDriving<DriveMode::Manual>();
}

} // namespace control::chassis

#endif // MANUAL_DRIVE_POLICY_IMPL_HPP