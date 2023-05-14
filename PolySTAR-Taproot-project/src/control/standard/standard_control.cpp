#ifdef TARGET_STANDARD

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "control/drivers/drivers_singleton.hpp"
#include "control/drivers/drivers.hpp"
#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
#include "subsystems/chassis/chassis_keyboard_drive_command.hpp"
#include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"
#include "subsystems/chassis/chassis_auto_drive_command.hpp"

// Turret includes
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_manual_aim_command.hpp"
#include "subsystems/turret/turret_left_aim_command.hpp"
#include "subsystems/turret/turret_right_aim_command.hpp"
#include "subsystems/turret/turret_mouse_aim_command.hpp"

// Feeder includes
#include "subsystems/feeder/feeder_subsystem.hpp"
#include "subsystems/feeder/feeder_feed_command.hpp"
#include "subsystems/feeder/feeder_reverse_command.hpp"

//Flywheel includes
#include "subsystems/flywheel/flywheel_subsystem.hpp"
#include "subsystems/flywheel/flywheel_fire_command.hpp"
#include "subsystems/flywheel/fire_command_group.hpp"

using src::DoNotUse_getDrivers;
using src::control::RemoteSafeDisconnectFunction;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using tap::control::HoldCommandMapping;
using tap::control::ToggleCommandMapping;
using tap::control::RemoteMapState;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
static src::driversFunc drivers = src::DoNotUse_getDrivers;

namespace control
{
/* define subsystems --------------------------------------------------------*/
chassis::ChassisSubsystem theChassis(drivers());
turret::TurretSubsystem theTurret(drivers());
feeder::FeederSubsystem theFeeder(drivers());
flywheel::FlywheelSubsystem theFlywheel(drivers());

/* define commands ----------------------------------------------------------*/
chassis::ChassisDriveCommand chassisDrive(&theChassis, drivers());
chassis::ChassisKeyboardDriveCommand chassisKeyboardDrive(&theChassis, drivers());
chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());
chassis::ChassisAutoDriveCommand chassisAutoDrive(&theChassis, drivers());

turret::TurretManualAimCommand turretManualAim(&theTurret, drivers());
turret::TurretLeftAimCommand turretLeftAim(&theTurret, drivers());
turret::TurretRightAimCommand turretRightAim(&theTurret, drivers());
turret::TurretMouseAimCommand turretMouseAim(&theTurret, drivers());

feeder::FeederFeedCommand feederForward(&theFeeder, drivers());
feeder::FeederReverseCommand feederReverse(&theFeeder, drivers());

flywheel::FlywheelFireCommand flywheelStart(&theFlywheel, drivers());
flywheel::FireCommandGroup fireCommandGroup(&theFlywheel, &theFeeder, drivers());
flywheel::FireEndCommandGroup fireEndCommandGroup(&theFlywheel, &theFeeder, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
// HoldCommandMapping feedFeeder(drivers(), {&feederForward}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
// HoldCommandMapping flywheelFire(drivers(), {&flywheelStart}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping rightAimTurret(drivers(), {&turretRightAim}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping leftAimTurret(drivers(), {&turretLeftAim}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping reverseFeeder(drivers(), {&feederReverse}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping remoteFireCommandGroup(drivers(), {&fireCommandGroup}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping mouseFireCommandGroup(drivers(), {&fireCommandGroup}, RemoteMapState(RemoteMapState::MouseButton::LEFT));
ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G}));
ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
ToggleCommandMapping toggleChassisAuto(drivers(), {&chassisAutoDrive}, RemoteMapState({Remote::Key::R}));

// HoldCommandMapping stopFiring(drivers(), {&fireEndCommandGroup}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&theChassis);
    drivers->commandScheduler.registerSubsystem(&theTurret);
    drivers->commandScheduler.registerSubsystem(&theFeeder);
    drivers->commandScheduler.registerSubsystem(&theFlywheel);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {
    theChassis.initialize();
    theTurret.initialize();
    theFeeder.initialize();
    theFlywheel.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *) {
    theChassis.setDefaultCommand(&chassisDrive);
    theTurret.setDefaultCommand(&turretManualAim);
    theFeeder.setDefaultCommand(&fireEndCommandGroup);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers) {
    drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers) {  
    drivers->commandMapper.addMap(&remoteFireCommandGroup);
    drivers->commandMapper.addMap(&mouseFireCommandGroup);
    // drivers->commandMapper.addMap(&stopFiring);
    drivers->commandMapper.addMap(&reverseFeeder);
    // drivers->commandMapper.addMap(&leftAimTurret);
    // drivers->commandMapper.addMap(&rightAimTurret);
    drivers->commandMapper.addMap(&toggleChassisDrive);
    drivers->commandMapper.addMap(&toggleChassisAuto);
    drivers->commandMapper.addMap(&turretMouseAimToggle);
}

void initSubsystemCommands(src::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    initializeSubsystems();
    registerStandardSubsystems(drivers);
    setDefaultStandardCommands(drivers);
    startStandardCommands(drivers);
    registerStandardIoMappings(drivers);
    char buffer[50];
    int nBytes = sprintf(buffer,"Initializing Standard\n");
    drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
}

}  // namespace control

#endif  // TARGET_STANDARD
