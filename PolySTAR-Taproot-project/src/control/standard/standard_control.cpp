#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "control/drivers/drivers_singleton.hpp"
#include "control/drivers/drivers.hpp"
#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
#include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"

// Turret includes
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_manual_aim_command.hpp"
#include "subsystems/turret/turret_debug_command.hpp"

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
chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());
turret::TurretManualAimCommand turretManualAim(&theTurret, drivers());
turret::TurretDebugCommand turretDebug(&theTurret, drivers());
feeder::FeederFeedCommand feederForward(&theFeeder, drivers());
feeder::FeederReverseCommand feederReverse(&theFeeder, drivers());
flywheel::FlywheelFireCommand flywheelStart(&theFlywheel, drivers());
flywheel::FireCommandGroup fireCommandGroupStart(&theFlywheel, &theFeeder, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
// HoldCommandMapping feedFeeder(drivers(), {&feederForward}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping reverseFeeder(drivers(), {&feederReverse}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
// HoldCommandMapping flywheelFire(drivers(), {&flywheelStart}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping remoteFireCommandGroup(drivers(), {&fireCommandGroupStart}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping mouseFireCommandGroup(drivers(), {&fireCommandGroupStart}, RemoteMapState(RemoteMapState::MouseButton::LEFT));

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
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *) {
    // drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers) {  
//    drivers->commandMapper.addMap(&feedFeeder);
   drivers->commandMapper.addMap(&reverseFeeder);
//    drivers->commandMapper.addMap(&flywheelFire);
   drivers->commandMapper.addMap(&remoteFireCommandGroup);
   drivers->commandMapper.addMap(&mouseFireCommandGroup);
}

void initSubsystemCommands(src::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    initializeSubsystems();
    registerStandardSubsystems(drivers);
    setDefaultStandardCommands(drivers);
    startStandardCommands(drivers);
    registerStandardIoMappings(drivers);
}

}  // namespace control
