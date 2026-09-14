#ifndef MANUAL_DRIVE_POLICY_IMPL_HPP
#define MANUAL_DRIVE_POLICY_IMPL_HPP

#include "manual_drive_policy.hpp"
#include "subsystems/chassis/utils/modes/drive_mode.hpp"
#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <typename Subsystem, SpinMode SpinMode>
ManualDrivePolicy<Subsystem, SpinMode>::ManualDrivePolicy(Subsystem* const chassis) :
    chassis_{chassis}
{
}

template <typename Subsystem, SpinMode SpinMode>
ManualDrivePolicy<Subsystem, SpinMode>::~ManualDrivePolicy() = default;

template <typename Subsystem, SpinMode SpinMode>
void ManualDrivePolicy<Subsystem, SpinMode>::initialize()
{
    chassis_->template initializeDriving<DriveMode::Manual, SpinMode>();
}

template <typename Subsystem, SpinMode SpinMode>
void ManualDrivePolicy<Subsystem, SpinMode>::execute()
{
    chassis_->template executeDriving<DriveMode::Manual, SpinMode>();
}

template <typename Subsystem, SpinMode SpinMode>
void ManualDrivePolicy<Subsystem, SpinMode>::end(const bool)
{
    chassis_->template endDriving<DriveMode::Manual, SpinMode>();
}

} // namespace control::chassis

#endif // MANUAL_DRIVE_POLICY_IMPL_HPP