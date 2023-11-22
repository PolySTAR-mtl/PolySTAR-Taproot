#ifndef MECANUM_DRIVE_COMMAND_HPP_
#define MECANUM_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "chassis_subsystem.hpp"

namespace control 
{
namespace chassis
{

class MecanumDriveCommand : public tap::control::Command
{
public:

    MecanumDriveCommand(ChassisSubsystem* const chassis, Drivers* drivers)
    : chassis(chassis), controlInterface(drivers)
    {
        if (chassis == nullptr)
        {
            return;
        }
        this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    }

    ~MecanumDriveCommand() = default;

    /**
     * Called once when the subsystem is added to the scheduler.
     */
    void initialize() override;

    /**
     * Returns the command name. Used by the CommandScheduler and for debugging purposes.
     */
    const char *getName() const { return "Mecanum Drive Command"; }

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void execute() override;

    /**
     * Will be called once when IsFinished() returns true or the command is interrupted
     */
    void end(bool interrupted) override;

    /**
     * Called periodically whenever the CommandScheduler runs. If it returns true, the end() method is called and the
     * command is removed from the CommandScheduler.
     */
    bool isFinished() const override;

private:
    ChassisSubsystem* chassis;
    src::control::ControlInterface controlInterface;

    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_BL = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_BR = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_FL = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_FR = tap::motor::MOTOR4;
};

} // namespace chassis

} // namespace control


#endif  // MECANUM_DRIVE_COMMAND_HPP_