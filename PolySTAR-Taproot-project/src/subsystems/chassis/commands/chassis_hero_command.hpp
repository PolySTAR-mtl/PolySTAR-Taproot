#ifndef CHASSIS_HERO_COMMAND_HPP_
#define CHASSIS_HERO_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../chassis_spin2win_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "../modes/chassis_operation_mode.hpp"


namespace control::chassis
{
class ChassisHeroCommand : public tap::control::Command 
{
public:

    ChassisHeroCommand(ChassisSpin2WinSubsystem *const chassis, src::Drivers *drivers, tap::motor::DjiMotor *turretYawMotor);

    ChassisHeroCommand(const ChassisHeroCommand &other) = delete;

    ChassisHeroCommand &operator=(const ChassisHeroCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis Hero command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    bool isMoving() const;

private:
    ChassisSpin2WinSubsystem *const chassis;

    src::Drivers *drivers;

    tap::motor::DjiMotor* turretYawMotor;

    friend struct ChassisOperationMode;
    
    ChassisOperationMode operationMode {};

    bool m_isMoving = false;

}; // ChassisHeroCommand
} // namespace control::chassis

#endif // CHASSIS_HERO_COMMAND_HPP_