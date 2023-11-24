#include "chassis_subsystem.hpp"
#include "tap/control/command.hpp"
#include "control/drivers/drivers.hpp"
using ControlWheels = src::control::ControlInterface;

class ChassisTankCommand : public tap::control::Command
{
public:

    ChassisTankCommand(ChassisSubsystem* const chassis, src::Drivers* drivers);

    ~ChassisTankCommand() = default;

    ChassisTankCommand(const ChassisTankCommand&) = delete;
    ChassisTankCommand& operator=(const ChassisTankCommand&) = delete;

    /**
     * Called once when the subsystem is added to the scheduler.
     */
    void initialize() override;

    /**
     * Returns the command name. Used by the CommandScheduler and for debugging purposes.
     */
    const char* getName() const;

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

private:
    ChassisSubsystem* const chassis_;
    src::Drivers* drivers_;
    static constexpr const char* const COMMAND_NAME = "Tank Drive Command";
    float MAX_RPM = 3500.0f;
};