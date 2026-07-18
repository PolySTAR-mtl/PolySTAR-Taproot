#ifdef TARGET_STANDARD

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/chassis/commands/chassis_drive_commands.hpp"
#include "subsystems/chassis/commands/chassis_spin2win_calibrate_IMU.hpp"

// Turret includes
#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/commands/turret_aim_commands.hpp"

// Feeder includes
#include "subsystems/feeder/core/feeder_position_subsystem.hpp"
#include "subsystems/feeder/commands/feeder_move_unjam_command.hpp"

//Flywheel includes
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "subsystems/flywheel/commands/flywheel_fire_commands.hpp"

#include "control/drivers/drivers_singleton.hpp"

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

using src::DoNotUse_getDrivers;

static src::driversFunc drivers = src::DoNotUse_getDrivers;
namespace control
{
/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor yawMotor(drivers(), tap::motor::MOTOR6, tap::can::CanBus::CAN_BUS1, true, "yaw motor");

chassis::OmniWheelsChassisSubsystem theChassis(drivers(), &yawMotor);
turret::TurretSubsystem theTurret(drivers(), &yawMotor);
feeder::FeederPositionSubsystem theFeeder(drivers());
flywheel::FlywheelSubsystem theFlywheel(drivers());

/* define commands ----------------------------------------------------------*/
/* chassis */
chassis::ManualDriveCommand chassisRelativeDrive(&theChassis, drivers());
chassis::ManualSpinDriveCommand chassisSpinDrive(&theChassis, drivers());
chassis::ChassisSpin2WinCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());

/* turret */
turret::ManualAimCommand turretManualAim(&theTurret, drivers());

/* feeder */
feeder::FeederMoveUnjamCommand feederMoveUnjam(&theFeeder, drivers());

/* flywheel */
flywheel::FireCommand flywheelStart(&theFlywheel, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
/* Controller mappings */
HoldRepeatCommandMapping feedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),true);
ToggleCommandMapping startFlywheel(drivers(), {&flywheelStart}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping toggleChassisSpin(drivers(), {&chassisSpinDrive}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

/* Mouse mappings */
ToggleCommandMapping mouseStartFlywheel(drivers(), {&flywheelStart}, RemoteMapState(RemoteMapState::MouseButton::RIGHT));
HoldRepeatCommandMapping mouseFeedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(RemoteMapState::MouseButton::LEFT),true);
// ToggleCommandMapping toggleClientAiming(drivers(), {&turretMouseNoSpin}, RemoteMapState({Remote::Key::F}));
// ToggleCommandMapping toggleChassisSpinKey(drivers(), {&chassisKeyboardDrive, &turretMouseAim}, RemoteMapState({Remote::Key::R}));
// ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
// ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G}));

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
    theChassis.setDefaultCommand(&chassisRelativeDrive);
    theTurret.setDefaultCommand(&turretManualAim);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers) {
    drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&feedFeeder);
    drivers->commandMapper.addMap(&startFlywheel);
    drivers->commandMapper.addMap(&toggleChassisSpin);
    drivers->commandMapper.addMap(&mouseStartFlywheel);
    drivers->commandMapper.addMap(&mouseFeedFeeder);
    // drivers->commandMapper.addMap(&toggleClientAiming);
    // drivers->commandMapper.addMap(&toggleChassisSpinKey);
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
    drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
}

}  // namespace control

#endif  // TARGET_STANDARD
