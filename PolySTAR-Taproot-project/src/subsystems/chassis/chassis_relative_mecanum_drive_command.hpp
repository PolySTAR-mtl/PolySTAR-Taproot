#pragma once

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisRelMecanumDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisRelMecanumDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers, tap::motor::DjiMotor *yawMotor);

    ChassisRelMecanumDriveCommand(const ChassisRelMecanumDriveCommand &other) = delete;

    ChassisRelMecanumDriveCommand &operator=(const ChassisRelMecanumDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis relative drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    tap::motor::DjiMotor *yawMotor;

    uint32_t prevDebugTime;

    ChassisSubsystem *const chassis;

    std::map<std::string, bool> keyboard_input;

    src::Drivers *drivers;
};  // ChassisRelMecanumDriveCommand

}  // namespace chassis

}  // namespace control

