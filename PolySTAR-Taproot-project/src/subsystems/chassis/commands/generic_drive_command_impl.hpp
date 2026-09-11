#ifndef GENERIC_DRIVE_COMMAND_IMPL_HPP
#define GENERIC_DRIVE_COMMAND_IMPL_HPP

#include "generic_drive_command.hpp"

namespace control::chassis
{

template <typename Subsystem, command_policy DrivePolicy>
GenericDriveCommand<Subsystem, DrivePolicy>::GenericDriveCommand(Subsystem* const chassis, src::Drivers* drivers)
    : tap::control::Command{},
        chassis_{chassis},
        drivers_{drivers},
        drivePolicy_{chassis}
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

template <typename Subsystem, command_policy DrivePolicy>
GenericDriveCommand<Subsystem, DrivePolicy>::~GenericDriveCommand() = default;

template <typename Subsystem, command_policy DrivePolicy>
void GenericDriveCommand<Subsystem, DrivePolicy>::initialize()
{
    drivePolicy_.initialize();
}

template <typename Subsystem, command_policy DrivePolicy>
void GenericDriveCommand<Subsystem, DrivePolicy>::execute()
{
    drivePolicy_.execute();
}

template <typename Subsystem, command_policy DrivePolicy>
const char* GenericDriveCommand<Subsystem, DrivePolicy>::getName() const
{
    return NAME;
}

template <typename Subsystem, command_policy DrivePolicy>
bool GenericDriveCommand<Subsystem, DrivePolicy>::isFinished() const
{
    return false;
}

template <typename Subsystem, command_policy DrivePolicy>
void GenericDriveCommand<Subsystem, DrivePolicy>::end(const bool interrupt)
{
    drivePolicy_.end(interrupt);
}

} // namespace control::chassis

#endif // GENERIC_DRIVE_COMMAND_IMPL_HPP