#ifdef TARGET_ICRA

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/drivers/drivers.hpp"
#include "control/drivers/drivers_singleton.hpp"
#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
#include "subsystems/chassis/chassis_keyboard_drive_command.hpp"
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/chassis/chassis_spin2win_command.hpp"

// Turret includes
#include "subsystems/turret/turret_manual_aim_command.hpp"
#include "subsystems/turret/turret_mouse_aim_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_test_bottomleft_command.hpp"
#include "subsystems/turret/turret_test_topright_command.hpp"

// Feeder includes
#include "subsystems/feeder/feeder_move_command.hpp"
#include "subsystems/feeder/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/feeder_position_subsystem.hpp"

// Flywheel includes
#include "subsystems/flywheel/flywheel_fire_command.hpp"
#include "subsystems/flywheel/flywheel_subsystem.hpp"

using src::DoNotUse_getDrivers;
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
static src::driversFunc drivers = src::DoNotUse_getDrivers;

namespace control
{
/* define subsystems --------------------------------------------------------*/
chassis::ChassisSubsystem theChassis(drivers());
turret::TurretSubsystem theTurret(drivers());
feeder::FeederPositionSubsystem theFeeder(drivers());
flywheel::FlywheelSubsystem theFlywheel(drivers());

/* define commands ----------------------------------------------------------*/
chassis::ChassisDriveCommand chassisDrive(&theChassis, drivers());
chassis::ChassisKeyboardDriveCommand chassisKeyboardDrive(&theChassis, drivers());
chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());
chassis::ChassisSpin2winCommand chassisSpin(&theChassis, drivers(), &(theTurret.getYawMotor()));


turret::TurretManualAimCommand turretManualAim(&theTurret, drivers());
turret::TurretTestBottomLeftCommand turretLeftAim(&theTurret, drivers()); // Used for tuning
turret::TurretTestTopRightCommand turretRightAim(&theTurret, drivers()); // Used for tuning
turret::TurretMouseAimCommand turretMouseAim(&theTurret, drivers());

feeder::FeederMoveUnjamCommand feederMoveUnjam(&theFeeder, drivers());

flywheel::FlywheelFireCommand flywheelStart(&theFlywheel, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
HoldRepeatCommandMapping feedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP), true);
HoldRepeatCommandMapping mouseFeedFeeder(drivers(), {&feederMoveUnjam}, RemoteMapState(RemoteMapState::MouseButton::LEFT), true);
ToggleCommandMapping toggleClientAiming(drivers(), {&chassisKeyboardDrive, &turretMouseAim}, RemoteMapState({Remote::Key::G}));
HoldCommandMapping spinChassis (drivers(), {&chassisSpin}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

/*-Testing commands-*/
// HoldCommandMapping mouseStartFlywheel(drivers(), {&flywheelStart}, RemoteMapState(RemoteMapState::MouseButton::RIGHT));
// ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G})); 
// ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
// HoldCommandMapping startFlywheel(drivers(), {&flywheelStart}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

/*-Only used for calibration-*/
// HoldCommandMapping rightAimTurret(drivers(), {&turretRightAim}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)); 
// HoldCommandMapping leftAimTurret(drivers(), {&turretLeftAim}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

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
    theChassis.setDefaultCommand(&chassisDrive);
    theTurret.setDefaultCommand(&turretManualAim);
    theFlywheel.setDefaultCommand(&flywheelStart);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers)
{
    drivers->commandMapper.addMap(&feedFeeder);
    drivers->commandMapper.addMap(&mouseFeedFeeder);
    drivers->commandMapper.addMap(&toggleClientAiming);
    // drivers->commandMapper.addMap(&startFlywheel);
    // drivers->commandMapper.addMap(&leftAimTurret);
    // drivers->commandMapper.addMap(&rightAimTurret);
    drivers->commandMapper.addMap(&spinChassis);
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
    int nBytes = sprintf(buffer, "Initializing Standard\n");
    drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart8,
        (uint8_t *)buffer,
        nBytes + 1);
}

}  // namespace control

#endif  // TARGET_ICRA
