#ifdef TARGET_SENTRY

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/safe_disconnect.hpp"

// Chassis includes
// #include "subsystems/chassis/chassis_auto_drive_command.hpp"
// #include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"
#include "subsystems/chassis/chassis_drivecommand.hpp"
#include "subsystems/chassis/chassis_subsystem.hpp"
// #include "subsystems/chassis/chassis_test_auto_drive_command.hpp"

// Turret includes
#include "subsystems/turret/turret_auto_aim_command.hpp"
#include "subsystems/turret/turret_test_bottomleft_command.hpp"
#include "subsystems/turret/turret_manual_aim_command.hpp"
#include "subsystems/turret/turret_mouse_aim_command.hpp"
#include "subsystems/turret/turret_test_topright_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_test_auto_aim_command.hpp"

// Feeder includes
#include "subsystems/feeder/feeder_velocity_subsystem.hpp"
#include "subsystems/feeder/feeder_auto_feed_command.hpp"
#include "subsystems/feeder/feeder_auto_feed_test_command.hpp"

#include "subsystems/feeder/feeder_position_subsystem.hpp" // Test with controller
#include "subsystems/feeder/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/feeder_move_command.hpp"

// Flywheel includes
#include "subsystems/flywheel/flywheel_fire_commands.hpp"
#include "subsystems/flywheel/flywheel_dji_subsystem.hpp"

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

chassis::ChassisSpin2WinSubsystem theChassis(drivers());
turret::TurretSubsystem theTurret(drivers(), &yawMotor);
flywheel::FlywheelSubsystem theFlywheel(drivers());
feeder::FeederVelocitySubsystem theFeeder(drivers());
feeder::FeederPositionSubsystem theTestFeeder(drivers());

/* define commands ----------------------------------------------------------*/
/* chassis ------------------------------------------------------------------*/
chassis::ChassisDriveCommand chassisDrive(&theChassis, drivers());
// chassis::ChassisAutoDriveCommand chassisAutoDrive(&theChassis, drivers());
//chassis::ChassisTestAutoDriveCommand chassisTestAutoDrive(&theChassis, drivers());
//chassis::ChassisKeyboardDriveCommand chassisKeyboardDrive(&theChassis, drivers());
// chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());

/* turret -------------------------------------------------------------------*/
turret::TurretManualAimCommand turretManualAim(&theTurret, drivers());
turret::TurretTestBottomLeftCommand turretLeftAim(&theTurret, drivers());
turret::TurretTestTopRightCommand turretRightAim(&theTurret, drivers());
turret::TurretMouseAimCommand turretMouseAim(&theTurret, drivers());
turret::TurretAutoAimCommand turretAutoAim(&theTurret, drivers());
turret::TurretTestAutoAimCommand turretTestAutoAim(&theTurret, drivers());

/* feeder -------------------------------------------------------------------*/
feeder::FeederMoveUnjamCommand feederMoveUnjam(&theTestFeeder, drivers());
feeder::FeederAutoFeedCommand feederAutoFeed(&theFeeder, drivers());
feeder::FeederAutoFeedTestCommand feederAutoFeedTest(&theFeeder, drivers());

/* flywheel -----------------------------------------------------------------*/
// flywheel::FlywheelAutoFireDjiCommand flywheelStartTest(&theFlywheel, drivers());
flywheel::FlywheelAutoFireCommand flywheelStart(&theFlywheel, drivers());
flywheel::FlywheelFireCommand flywheelStartManual(&theFlywheel, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
/*-Ammo Booster-*/
HoldRepeatCommandMapping feedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),true);
/*-Flywheel-*/
// HoldCommandMapping startFlywheel(drivers(), {&flywheelStartTest}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping startFlywheelManual(drivers(), {&flywheelStartManual}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
/*-Turret-*/
ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
/*-Chassis-*/
// ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G}));
/*-Auto commands*/
// HoldCommandMapping toggleAutoCommands(drivers(), {&chassisAutoDrive, &turretAutoAim, &feederAutoFeed, &flywheelStart}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
// HoldCommandMapping toggleAutoTestCommands(drivers(), {&chassisTestAutoDrive, &turretTestAutoAim, &feederAutoFeedTest}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

/*-Only used for calibration-*/
// HoldCommandMapping rightAimTurret(drivers(), {&turretRightAim}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)); 
// HoldCommandMapping leftAimTurret(drivers(), {&turretLeftAim}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&theChassis);
    drivers->commandScheduler.registerSubsystem(&theTurret);
    // drivers->commandScheduler.registerSubsystem(&theTestFeeder);
    drivers->commandScheduler.registerSubsystem(&theFeeder);
    drivers->commandScheduler.registerSubsystem(&theFlywheel);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    theChassis.initialize();
    theTurret.initialize();
    theFeeder.initialize();
    // theTestFeeder.initialize();
    theFlywheel.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *)
{
    // theChassis.setDefaultCommand(&chassisAutoDrive);
    theChassis.setDefaultCommand(&chassisRelativeDrive);
    theTurret.setDefaultCommand(&turretManualAim);
    theFlywheel.setDefaultCommand(&flywheelStart);
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
    drivers->commandMapper.addMap(&turretMouseAimToggle);
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
