#ifndef GENERIC_DRIVE_COMMAND_IMPL_HPP
#define GENERIC_DRIVE_COMMAND_IMPL_HPP

#include "generic_drive_command.hpp"

namespace control::chassis
{

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::GenericDriveCommand(Subsystem* const chassis, src::Drivers* drivers)
    : tap::control::Command{},
        chassis_{chassis},
        drivers_{drivers},
        drivePolicy_{chassis},
        spinPolicy_{chassis}
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::~GenericDriveCommand() = default;

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
void GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::initialize()
{
    drivePolicy_.initialize();
}

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
void GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::execute()
{
    drivePolicy_.execute();
    spinPolicy_.execute();
    chassis_->updateDesiredOutput();
}

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
const char* GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::getName() const
{
    return NAME;
}

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
bool GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::isFinished() const
{
    return false;
}

template <typename Subsystem, command_policy DrivePolicy, command_policy SpinPolicy>
void GenericDriveCommand<Subsystem, DrivePolicy, SpinPolicy>::end(const bool interrupt)
{
    drivePolicy_.end(interrupt);
}

} // namespace control::chassis

#endif // GENERIC_DRIVE_COMMAND_IMPL_HPP