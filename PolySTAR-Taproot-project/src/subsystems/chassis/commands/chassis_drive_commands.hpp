#ifndef CHASSIS_DRIVE_COMMANDS
#define CHASSIS_DRIVE_COMMANDS

#include "generic_drive_command.hpp"

#include "subsystems/chassis/utils/auto_drive_policy.hpp"
#include "subsystems/chassis/utils/manual_drive_policy.hpp"
#include "subsystems/chassis/core/chassis_subsystem.hpp"

namespace control::chassis
{

using AutoDriveCommand = 
    GenericDriveCommand<MecanumChassisSubsystem, AutoDrivePolicy<MecanumChassisSubsystem>>;

using ManualDriveCommand = 
    GenericDriveCommand<OmniWheelsChassisSubsystem, ManualDrivePolicy<OmniWheelsChassisSubsystem>>;

} // namespace control::chassis

#endif // CHASSIS_DRIVE_COMMANDS