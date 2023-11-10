#ifndef MECANUM_DRIVE_COMMAND_HPP_
#define MECANUM_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

namespace control 
{
namespace chassis
{

class MecanumDriveCommand : public tap::control::Command
{
public:

    MecanumDriveCommand();

    ~MecanumDriveCommand() = default;

    /**
     * Called once when the subsystem is added to the scheduler.
     */
    void initialize() override;

    /**
     * Returns the command name. Used by the CommandScheduler and for debugging purposes.
     */
    const char *getName() const { return "Command Name"; }

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void execute() override;

    /**
     * Will be called once when IsFinished() returns true or the command is interrupted
     */
    void end(bool) override;

    /**
     * Called periodically whenever the CommandScheduler runs. If it returns true, the end() method is called and the
     * command is removed from the CommandScheduler.
     */
    bool isFinished() const override;

};

} // namespace chassis

} // namespace control


#endif  // MECANUM_DRIVE_COMMAND_HPP_