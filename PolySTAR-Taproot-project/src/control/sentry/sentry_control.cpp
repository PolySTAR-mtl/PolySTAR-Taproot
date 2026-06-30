#ifdef TARGET_SENTRY

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/commands/chassis_sentry_command.hpp"
#include "subsystems/chassis/commands/chassis_spin2win_command.hpp"
#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"

// Turret includes
#include "subsystems/turret/commands/turret_sentry_command.hpp"
#include "subsystems/turret/commands/turret_spin2win_command.hpp"
#include "subsystems/turret/core/turret_subsystem.hpp"

// Feeder includes
#include "subsystems/feeder/core/feeder_position_subsystem.hpp"
#include "subsystems/feeder/core/feeder_velocity_subsystem.hpp"
#include "subsystems/feeder/commands/feeder_feed_commands.hpp"
#include "subsystems/feeder/commands/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/commands/feeder_move_command.hpp"

// Flywheel includes
#include "subsystems/flywheel/commands/flywheel_fire_commands.hpp"
#include "subsystems/flywheel/core/flywheel_dji_subsystem.hpp"

// Motor includes
#include "control/motor_control.hpp"

using src::control::RemoteSafeDisconnectFunction;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using tap::control::HoldCommandMapping;
using tap::control::HoldRepeatCommandMapping;
using tap::control::RemoteMapState;
using tap::control::ToggleCommandMapping;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */

namespace control
{
/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor yawMotor(drivers(), tap::motor::MOTOR6, tap::can::CanBus::CAN_BUS1, true, "yaw motor");

chassis::MecanumChassisSubsystem theChassis(drivers());
turret::TurretSubsystem theTurret(drivers(), &yawMotor);
flywheel::FlywheelSubsystem theFlywheel(drivers());
feeder::FeederPositionSubsystem theFeeder(drivers());
feeder::FeederVelocitySubsystem theFeederVelocity(drivers(), drivers());

/* define commands ----------------------------------------------------------*/
/* chassis ------------------------------------------------------------------*/
// chassis::ChassisSpin2winDriveCommand chassisDrive(&theChassis, drivers(), &yawMotor);
chassis::ChassisSentryDriveCommand chassisAutoDrive(&theChassis, drivers());
// chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());

/* turret -------------------------------------------------------------------*/
// turret::TurretSpin2WinAimCommand turretManualAim(&theTurret, drivers());
turret::TurretSentryAimCommand turretAutoAim(&theTurret, drivers());

/* feeder -------------------------------------------------------------------*/
// feeder::FeederMoveUnjamCommand feederMoveUnjam(&theFeeder, drivers());
feeder::FeederAutoFeedCommand feederAutoFeed(&theFeederVelocity, drivers());

/* flywheel -----------------------------------------------------------------*/
// flywheel::FlywheelAutoFireDjiCommand flywheelStartTest(&theFlywheel, drivers());
flywheel::FlywheelAutoFireCommand flywheelStart(&theFlywheel, drivers());
// flywheel::FlywheelFireCommand flywheelStartManual(&theFlywheel, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
/*-Ammo Booster-*/
HoldRepeatCommandMapping feedFeeder(drivers(), {&feederAutoFeed}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),true);
/*-Flywheel-*/
// HoldCommandMapping startFlywheel(drivers(), {&flywheelStartTest}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
ToggleCommandMapping startFlywheelManual(drivers(), {&flywheelStart}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
/*-Turret-*/
// ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
/*-Chassis-*/
// ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G}));
/*-Auto commands*/
// HoldCommandMapping toggleAutoCommands(drivers(), {&chassisAutoDrive, &turretAutoAim, &feederAutoFeed, &flywheelStart}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
// HoldCommandMapping toggleAutoTestCommands(drivers(), {&chassisTestAutoDrive, &turretTestAutoAim, &feederAutoFeedTest}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&theChassis);
    drivers->commandScheduler.registerSubsystem(&theTurret);
    drivers->commandScheduler.registerSubsystem(&theFeeder);
    drivers->commandScheduler.registerSubsystem(&theFlywheel);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    theChassis.initialize();
    theTurret.initialize();
    theFeeder.initialize();
    theFlywheel.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *)
{
    theChassis.setDefaultCommand(&chassisAutoDrive);
    // theChassis.setDefaultCommand(&chassisDrive);
    theTurret.setDefaultCommand(&turretAutoAim);
    // theFlywheel.setDefaultCommand(&flywheelStart);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers)
{
    /*-Ammo Booster-*/
    drivers->commandMapper.addMap(&feedFeeder);
    
    /*-Flywheel-*/
    // drivers->commandMapper.addMap(&startFlywheel);
    drivers->commandMapper.addMap(&startFlywheelManual);
    /*-Turret-*/
    // drivers->commandMapper.addMap(&leftAimTurret);
    // drivers->commandMapper.addMap(&rightAimTurret);
    // drivers->commandMapper.addMap(&turretMouseAimToggle);
    /*-Chassis-*/
    // drivers->commandMapper.addMap(&toggleChassisDrive);
    // drivers->commandMapper.addMap(&toggleAutoTestCommands);
    // drivers->commandMapper.addMap(&toggleAutoCommands);
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
    int nBytes = sprintf(buffer, "Initializing Sentry\n");
    drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart8,
        (uint8_t *)buffer,
        nBytes + 1);
}

}  // namespace control

#endif  // TARGET_SENTRY
