#ifdef TARGET_SENTRY

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/chassis/commands/chassis_drive_commands.hpp"

// Turret includes
#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/commands/turret_aim_commands.hpp"

// Feeder includes
#include "subsystems/feeder/core/feeder_velocity_subsystem.hpp"
#include "subsystems/feeder/commands/feeder_feed_commands.hpp"
#include "subsystems/feeder/core/feeder_position_subsystem.hpp"
#include "subsystems/feeder/commands/feeder_move_unjam_command.hpp"

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

chassis::MecanumChassisSubsystem theChassis(drivers(), &yawMotor);
turret::TurretSubsystem theTurret(drivers(), &yawMotor);
flywheel::FlywheelDjiSubsystem theFlywheel(drivers());
feeder::FeederVelocitySubsystem theVelocityFeeder(drivers(), drivers());
feeder::FeederPositionSubsystem thePositionFeeder(drivers());


/* define commands ----------------------------------------------------------*/

/* chassis ------------------------------------------------------------------*/
chassis::SentryManualDriveCommand chassisDrive(&theChassis, drivers());
chassis::SentryAutoDriveCommand chassisAutoDrive(&theChassis, drivers());
// chassis::ChassisKeyboardDriveCommand chassisKeyboardDrive(&theChassis, drivers());

/* turret -------------------------------------------------------------------*/
turret::ManualAimCommand turretManualAim(&theTurret, drivers());
// turret::TurretMouseAimCommand turretMouseAim(&theTurret, drivers());
turret::AutoAimCommand turretAutoAim(&theTurret, drivers());

/* feeder -------------------------------------------------------------------*/
feeder::FeederMoveUnjamCommand feederMoveUnjam(&thePositionFeeder, drivers());
feeder::AutoFeedCommand feederAutoFeed(&theVelocityFeeder, drivers());

/* flywheel -----------------------------------------------------------------*/
flywheel::AutoFireCommand flywheelAutoStart(&theFlywheel, drivers());
flywheel::FireCommand flywheelStartManual(&theFlywheel, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
/* Controller mappings */
HoldRepeatCommandMapping feedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),true);
ToggleCommandMapping startFlywheel(drivers(), {&flywheelStartManual}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping toggleAutoCommands(drivers(), {&chassisAutoDrive, &turretAutoAim, &feederAutoFeed, &flywheelAutoStart}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

/* Mouse mappings */
// ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
// ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G}));


/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&theChassis);
    drivers->commandScheduler.registerSubsystem(&theTurret);
    drivers->commandScheduler.registerSubsystem(&theVelocityFeeder);
    drivers->commandScheduler.registerSubsystem(&thePositionFeeder);
    drivers->commandScheduler.registerSubsystem(&theFlywheel);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    theChassis.initialize();
    theTurret.initialize();
    theVelocityFeeder.initialize();
    thePositionFeeder.initialize();
    theFlywheel.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *)
{
    theChassis.setDefaultCommand(&chassisDrive);
    theTurret.setDefaultCommand(&turretManualAim);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers)
{
    drivers->commandMapper.addMap(&feedFeeder);
    drivers->commandMapper.addMap(&startFlywheel);
    // drivers->commandMapper.addMap(&turretMouseAimToggle);
    // drivers->commandMapper.addMap(&toggleChassisDrive);
    drivers->commandMapper.addMap(&toggleAutoCommands);
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
