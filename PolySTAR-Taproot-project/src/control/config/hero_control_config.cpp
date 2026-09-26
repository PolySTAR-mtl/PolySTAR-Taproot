#include "control/config/control_config.hpp"

namespace control
{

ControlConfig<target::RobotTarget::Hero>::ControlConfig(src::Drivers *drivers)
    : drivers_{drivers}
    , yawMotor{drivers, tap::motor::MOTOR6, tap::can::CanBus::CAN_BUS1, true, "yaw motor"}
    , theChassis{drivers, &yawMotor}
    , theTurret{drivers, &yawMotor}
    , theFeeder{drivers}
    , theFlywheel{drivers}
    , chassisRelativeDrive{&theChassis, drivers}
    , chassisSpinDrive{&theChassis, drivers}
    , chassisImuCalibrate{&theChassis, drivers}
    , turretManualAim{&theTurret, drivers}
    , turretManualSpinAim{&theTurret, drivers}
    , feederMoveUnjam{&theFeeder, drivers}
    , flywheelStart{&theFlywheel, drivers}
    , remoteSafeDisconnectFunction{drivers}
    , feedFeeder{
          drivers,
          {&feederMoveUnjam},
          tap::control::RemoteMapState{
              tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
              tap::communication::serial::Remote::SwitchState::UP},
          true}
    , startFlywheel{
          drivers,
          {&flywheelStart},
          tap::control::RemoteMapState{
              tap::communication::serial::Remote::Switch::LEFT_SWITCH,
              tap::communication::serial::Remote::SwitchState::DOWN}}
    , toggleChassisSpin{
          drivers,
          {&chassisSpinDrive, &turretManualSpinAim},
          tap::control::RemoteMapState{
              tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
              tap::communication::serial::Remote::SwitchState::DOWN}}
{
}

void ControlConfig<target::RobotTarget::Hero>::initialize()
{
    drivers_->commandScheduler.setSafeDisconnectFunction(
        &remoteSafeDisconnectFunction);
    initializeSubsystems();
    registerSubsystems();
    setDefaultCommands();
    startCommands();
    registerIoMappings();
    char buffer[50];
    const int nBytes = sprintf(buffer, "Initializing Hero\n");
    drivers_->uart.write(
        tap::communication::serial::Uart::UartPort::Uart8,
        reinterpret_cast<uint8_t *>(buffer),
        nBytes + 1);
}

void ControlConfig<target::RobotTarget::Hero>::registerSubsystems()
{
    drivers_->commandScheduler.registerSubsystem(&theChassis);
    drivers_->commandScheduler.registerSubsystem(&theTurret);
    drivers_->commandScheduler.registerSubsystem(&theFeeder);
    drivers_->commandScheduler.registerSubsystem(&theFlywheel);
}

void ControlConfig<target::RobotTarget::Hero>::initializeSubsystems()
{
    theChassis.initialize();
    theTurret.initialize();
    theFeeder.initialize();
    theFlywheel.initialize();
}

void ControlConfig<target::RobotTarget::Hero>::setDefaultCommands()
{
    theChassis.setDefaultCommand(&chassisRelativeDrive);
    theTurret.setDefaultCommand(&turretManualAim);
}

void ControlConfig<target::RobotTarget::Hero>::startCommands()
{
    drivers_->commandScheduler.addCommand(&chassisImuCalibrate);
}

void ControlConfig<target::RobotTarget::Hero>::registerIoMappings()
{
    drivers_->commandMapper.addMap(&feedFeeder);
    drivers_->commandMapper.addMap(&startFlywheel);
    drivers_->commandMapper.addMap(&toggleChassisSpin);
}

}  // namespace control