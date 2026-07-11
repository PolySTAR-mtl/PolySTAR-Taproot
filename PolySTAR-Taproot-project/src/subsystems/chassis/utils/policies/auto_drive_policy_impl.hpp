#ifndef AUTO_DRIVE_POLICY_IMPL_HPP
#define AUTO_DRIVE_POLICY_IMPL_HPP

#include "auto_drive_policy.hpp"
#include "subsystems/chassis/utils/modes/drive_mode.hpp"

namespace control::chassis
{

template <typename Subsystem, SpinMode SpinMode>
AutoDrivePolicy<Subsystem, SpinMode>::AutoDrivePolicy(Subsystem* const chassis) :
    chassis_{chassis}
{
}

template <typename Subsystem, SpinMode SpinMode>
AutoDrivePolicy<Subsystem, SpinMode>::~AutoDrivePolicy() = default;

template <typename Subsystem, SpinMode SpinMode>
void AutoDrivePolicy<Subsystem, SpinMode>::initialize()
{
    chassis_->template initializeDriving<DriveMode::Auto, SpinMode>();
}

template <typename Subsystem, SpinMode SpinMode>
void AutoDrivePolicy<Subsystem, SpinMode>::execute()
{
    chassis_->template executeDriving<DriveMode::Auto, SpinMode>();
}

template <typename Subsystem, SpinMode SpinMode>
void AutoDrivePolicy<Subsystem, SpinMode>::end(const bool)
{
    chassis_->template endDriving<DriveMode::Auto, SpinMode>();
}

} // namespace control::chassis

#endif // AUTO_DRIVE_POLICY_IMPL_HPP