#ifndef CHASSIS_START_COMMAND_H
#define CHASSIS_START_COMMAND_H

#include "tap/control/command.hpp"
#include "control/drivers/drivers.hpp"

#include "chassis_subsystem.hpp"

class ChassisMoveCommand : public tap::control::Command{
public:
    ChassisMoveCommand(ChassisSubsystem *const chassisSubsystem, src::Drivers* driver);
    ~ChassisMoveCommand() = default;

    void initialize() override;
    const char *getName() const;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;

private:
    static constexpr float MAX_RPM = 4000;

    ChassisSubsystem *const chassisSubsystem;
    src::Drivers *drivers;
};

#endif //CHASSIS_START_COMMAND_H