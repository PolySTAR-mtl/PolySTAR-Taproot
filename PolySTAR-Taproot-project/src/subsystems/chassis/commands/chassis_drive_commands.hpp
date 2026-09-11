#ifndef CHASSIS_DRIVE_COMMANDS
#define CHASSIS_DRIVE_COMMANDS

#include "generic_drive_command.hpp"

#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/chassis/utils/policies/auto_drive_policy.hpp"
#include "subsystems/chassis/utils/policies/manual_drive_policy.hpp"

namespace control::chassis
{

using SentryAutoDriveCommand =
    GenericDriveCommand<MecanumChassisSubsystem, AutoDrivePolicy<MecanumChassisSubsystem, SpinMode::NoSpin>>;

using SentryManualDriveCommand =
    GenericDriveCommand<MecanumChassisSubsystem, ManualDrivePolicy<MecanumChassisSubsystem, SpinMode::NoSpin>>;

using ManualDriveCommand =
    GenericDriveCommand<OmniWheelsChassisSubsystem, ManualDrivePolicy<OmniWheelsChassisSubsystem, SpinMode::NoSpin>>;

using ManualSpinDriveCommand =
    GenericDriveCommand<OmniWheelsChassisSubsystem, ManualDrivePolicy<OmniWheelsChassisSubsystem, SpinMode::Spin>>;

} // namespace control::chassis

#endif // CHASSIS_DRIVE_COMMANDS