#ifndef GENERIC_DRIVE_COMMAND_HPP
#define GENERIC_DRIVE_COMMAND_HPP

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/concepts/command_policy.hpp"
#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <typename Subsystem, command_policy DrivePolicy>
class GenericDriveCommand : public tap::control::Command
{
public:
    GenericDriveCommand(Subsystem* const chassis, src::Drivers* drivers);

    ~GenericDriveCommand();

    GenericDriveCommand(const GenericDriveCommand& other) = delete;

    GenericDriveCommand& operator=(const GenericDriveCommand& other) = delete;

    void initialize() override;

    void execute() override;

    const char* getName() const override;

    bool isFinished() const override;

    void end(const bool interrupt) override;

private:
    static constexpr const char* NAME = "chassis drive command";
    Subsystem* const chassis_;
    src::Drivers* drivers_;
    DrivePolicy drivePolicy_;
};

}

#include "generic_drive_command_impl.hpp"

#endif // GENERIC_DRIVE_COMMAND_HPP