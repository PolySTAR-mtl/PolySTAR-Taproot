#ifdef TARGET_HERO

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "control/drivers/drivers_singleton.hpp"
#include "control/drivers/drivers.hpp"
#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
#include "subsystems/chassis/chassis_keyboard_drive_command.hpp"
#include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"

// Turret includes
#include "subsystems/turret/turret_rpm_subsystem.hpp"
#include "subsystems/turret/turret_rpm_aim_command.hpp"
#include "subsystems/turret/turret_left_aim_command.hpp"
#include "subsystems/turret/turret_rpm_mouse_aim_command.hpp"
// Feeder includes
#include "subsystems/feeder/feeder_subsystem.hpp"
#include "subsystems/feeder/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/feeder_move_command.hpp"
#include "subsystems/feeder/feeder_subsystem_legacy.hpp"

//Flywheel includes
#include "subsystems/flywheel/flywheel_subsystem.hpp"
#include "subsystems/flywheel/flywheel_fire_command.hpp"
#include "subsystems/feeder/feeder_feed_command.hpp"

using src::DoNotUse_getDrivers;
using src::control::RemoteSafeDisconnectFunction;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using tap::control::HoldCommandMapping;
using tap::control::HoldRepeatCommandMapping;
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
turret::TurretRpmSubsystem theTurret(drivers());
feeder::FeederSubsystem topFeeder(drivers());
feeder::FeederSubsystemLegacy bottomFeeder(drivers());
flywheel::FlywheelSubsystem theFlywheel(drivers());

/* define commands ----------------------------------------------------------*/
chassis::ChassisDriveCommand chassisDrive(&theChassis, drivers());
chassis::ChassisKeyboardDriveCommand chassisKeyboardDrive(&theChassis, drivers());
chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());

turret::TurretRpmAimCommand turretManualAim(&theTurret, drivers());
turret::TurretRpmMouseAimCommand turretMouseAim(&theTurret, drivers());

feeder::FeederMoveUnjamCommand feederMoveUnjam(&topFeeder, drivers());
feeder::FeederMoveCommand feederMove(&topFeeder);
feeder::FeederFeedCommand feedFeederLegacy(&bottomFeeder, drivers());
flywheel::FlywheelFireCommand flywheelStart(&theFlywheel, drivers());


/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
HoldRepeatCommandMapping mouseFeedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(RemoteMapState::MouseButton::LEFT),true);
HoldCommandMapping mouseStartFlywheel(drivers(), {&flywheelStart}, RemoteMapState(RemoteMapState::MouseButton::RIGHT));
HoldCommandMapping feedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping startFlywheel(drivers(), {&flywheelStart, &feedFeederLegacy}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
ToggleCommandMapping toggleClientAiming(drivers(), {&chassisKeyboardDrive,&turretMouseAim}, RemoteMapState({Remote::Key::G}));

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&theChassis);
    drivers->commandScheduler.registerSubsystem(&theTurret);
    drivers->commandScheduler.registerSubsystem(&topFeeder);
    drivers->commandScheduler.registerSubsystem(&theFlywheel);
    drivers->commandScheduler.registerSubsystem(&bottomFeeder);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {
    theChassis.initialize();
    theTurret.initialize();
    topFeeder.initialize();
    theFlywheel.initialize();
    bottomFeeder.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *) {
    theChassis.setDefaultCommand(&chassisDrive);
    theTurret.setDefaultCommand(&turretManualAim);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers) {
    drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers) {  
    drivers->commandMapper.addMap(&feedFeeder);
    drivers->commandMapper.addMap(&mouseFeedFeeder);
    drivers->commandMapper.addMap(&mouseStartFlywheel);
    drivers->commandMapper.addMap(&startFlywheel);
    drivers->commandMapper.addMap(&toggleClientAiming);
}

void initSubsystemCommands(src::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    initializeSubsystems();
    registerStandardSubsystems(drivers);
    setDefaultStandardCommands(drivers);
    startStandardCommands(drivers);
    registerStandardIoMappings(drivers);
    // char buffer[50];
    // int nBytes = sprintf(buffer,"Initializing Hero\n");
    //drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
}

}  // namespace control

#endif  // TARGET_HERO
