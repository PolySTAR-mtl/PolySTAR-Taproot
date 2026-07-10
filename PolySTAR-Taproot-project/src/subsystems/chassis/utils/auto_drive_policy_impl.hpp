#ifndef AUTO_DRIVE_POLICY_IMPL_HPP
#define AUTO_DRIVE_POLICY_IMPL_HPP

#include "auto_drive_policy.hpp"
#include "drive_mode.hpp"

namespace control::chassis
{

template <typename Subsystem>
AutoDrivePolicy<Subsystem>::AutoDrivePolicy(Subsystem* const chassis) : 
    chassis_{chassis}
{

}

template <typename Subsystem>
AutoDrivePolicy<Subsystem>::~AutoDrivePolicy() = default;

template <typename Subsystem>
void AutoDrivePolicy<Subsystem>::initialize()
{
    chassis_->template initializeDriving<DriveMode::Auto>();
}

template <typename Subsystem>
void AutoDrivePolicy<Subsystem>::execute()
{
    chassis_->template executeDriving<DriveMode::Auto>();
}

template <typename Subsystem>
void AutoDrivePolicy<Subsystem>::end(const bool interrupt)
{
    chassis_->template endDriving<DriveMode::Auto>();
}


} // namespace control::chassis
 

#endif // AUTO_DRIVE_POLICY_IMPL_HPP