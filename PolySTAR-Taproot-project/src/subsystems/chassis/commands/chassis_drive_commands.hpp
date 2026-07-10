#ifndef CHASSIS_DRIVE_COMMANDS
#define CHASSIS_DRIVE_COMMANDS

#include "generic_drive_command.hpp"

#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/chassis/utils/policies/auto_drive_policy.hpp"
#include "subsystems/chassis/utils/policies/manual_drive_policy.hpp"
#include "subsystems/chassis/utils/policies/spin_policy.hpp"
#include "subsystems/chassis/utils/policies/no_spin_policy.hpp"

namespace control::chassis
{

using SentryAutoDriveCommand =
    GenericDriveCommand<MecanumChassisSubsystem, AutoDrivePolicy<MecanumChassisSubsystem>, NoSpinPolicy<MecanumChassisSubsystem>>;

using SentryManualDriveCommand =
    GenericDriveCommand<MecanumChassisSubsystem, ManualDrivePolicy<MecanumChassisSubsystem>, NoSpinPolicy<MecanumChassisSubsystem>>;

using ManualDriveCommand =
    GenericDriveCommand<OmniWheelsChassisSubsystem, ManualDrivePolicy<OmniWheelsChassisSubsystem>, NoSpinPolicy<OmniWheelsChassisSubsystem>>;

using ManualSpinDriveCommand =
    GenericDriveCommand<OmniWheelsChassisSubsystem, ManualDrivePolicy<OmniWheelsChassisSubsystem>, SpinPolicy<OmniWheelsChassisSubsystem>>;

} // namespace control::chassis

#endif // CHASSIS_DRIVE_COMMANDS