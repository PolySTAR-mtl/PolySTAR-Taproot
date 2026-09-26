#include "control/config/control_config.hpp"

namespace control
{

ControlConfig<target::RobotTarget::Sentry>::ControlConfig(src::Drivers *drivers)
    : drivers_{drivers}
    , yawMotor{drivers, tap::motor::MOTOR6, tap::can::CanBus::CAN_BUS1, true, "yaw motor"}
    , theChassis{drivers, &yawMotor}
    , theTurret{drivers, &yawMotor}
    , theFlywheel{drivers}
    , theVelocityFeeder{drivers, drivers}
    , thePositionFeeder{drivers}
    , chassisDrive{&theChassis, drivers}
    , chassisAutoDrive{&theChassis, drivers}
    , turretManualAim{&theTurret, drivers}
    , turretAutoAim{&theTurret, drivers}
    , feederMoveUnjam{&thePositionFeeder, drivers}
    , feederAutoFeed{&theVelocityFeeder, drivers}
    , flywheelAutoStart{&theFlywheel, drivers}
    , flywheelStartManual{&theFlywheel, drivers}
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
          {&flywheelStartManual},
          tap::control::RemoteMapState{
              tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
              tap::communication::serial::Remote::SwitchState::DOWN}}
    , toggleAutoCommands{
          drivers,
          {&chassisAutoDrive, &turretAutoAim, &feederAutoFeed, &flywheelAutoStart},
          tap::control::RemoteMapState{
              tap::communication::serial::Remote::Switch::LEFT_SWITCH,
              tap::communication::serial::Remote::SwitchState::DOWN}}
{
}

void ControlConfig<target::RobotTarget::Sentry>::initialize()
{
    drivers_->commandScheduler.setSafeDisconnectFunction(
        &remoteSafeDisconnectFunction);
    initializeSubsystems();
    registerSubsystems();
    setDefaultCommands();
    startCommands();
    registerIoMappings();
    char buffer[50];
    const int nBytes = sprintf(buffer, "Initializing Sentry\n");
    drivers_->uart.write(
        tap::communication::serial::Uart::UartPort::Uart8,
        reinterpret_cast<uint8_t *>(buffer),
        nBytes + 1);
}

void ControlConfig<target::RobotTarget::Sentry>::registerSubsystems()
{
    drivers_->commandScheduler.registerSubsystem(&theChassis);
    drivers_->commandScheduler.registerSubsystem(&theTurret);
    drivers_->commandScheduler.registerSubsystem(&theVelocityFeeder);
    drivers_->commandScheduler.registerSubsystem(&thePositionFeeder);
    drivers_->commandScheduler.registerSubsystem(&theFlywheel);
}

void ControlConfig<target::RobotTarget::Sentry>::initializeSubsystems()
{
    theChassis.initialize();
    theTurret.initialize();
    theVelocityFeeder.initialize();
    thePositionFeeder.initialize();
    theFlywheel.initialize();
}

void ControlConfig<target::RobotTarget::Sentry>::setDefaultCommands()
{
    theChassis.setDefaultCommand(&chassisDrive);
    theTurret.setDefaultCommand(&turretManualAim);
}

void ControlConfig<target::RobotTarget::Sentry>::startCommands() {}

void ControlConfig<target::RobotTarget::Sentry>::registerIoMappings()
{
    drivers_->commandMapper.addMap(&feedFeeder);
    drivers_->commandMapper.addMap(&startFlywheel);
    drivers_->commandMapper.addMap(&toggleAutoCommands);
}

}  // namespace control