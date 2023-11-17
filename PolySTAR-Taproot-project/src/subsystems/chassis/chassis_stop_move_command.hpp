#ifndef CHASSIS_STOP_COMMAND_H
#define CHASSIS_STOP_COMMAND_H

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"

class ChassisStopCommand : public tap::control::Command{
public:
    ChassisStopCommand(ChassisSubsystem &chassisSubsystem);
    ~ChassisStopCommand() = default;

    void initialize() override;
    const char *getName() const;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;

private:
    ChassisSubsystem &chassisSubsystem;
};

#endif //CHASSIS_STOP_COMMAND_H