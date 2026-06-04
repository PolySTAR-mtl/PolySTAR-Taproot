#ifdef TARGET_SPIN_TO_WIN

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "control/safe_disconnect.hpp"

// Chassis includes
#include "subsystems/chassis/chassis_spin2win_subsystem.hpp"
#include "subsystems/chassis/chassis_relative_drive_command.hpp"
#include "subsystems/chassis/chassis_spin2win_command.hpp"
#include "subsystems/chassis/chassis_spin2win_keyboard_command.hpp"
#include "subsystems/chassis/chassis_calibrate_IMU_command.hpp"

// Turret includes
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_manual_aim_command.hpp"
#include "subsystems/turret/turret_mouse_aim_command.hpp"
#include "subsystems/turret/turret_stable_manual_aim_command.hpp"
#include "subsystems/turret/turret_stable_mouse_aim_command.hpp"
#include "subsystems/turret/turret_test_bottomleft_command.hpp"
#include "subsystems/turret/turret_test_topright_command.hpp"

// Feeder includes
#include "subsystems/feeder/feeder_position_subsystem.hpp"
#include "subsystems/feeder/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/feeder_move_command.hpp"

//Flywheel includes
#include "subsystems/flywheel/flywheel_subsystem.hpp"
#include "subsystems/flywheel/flywheel_fire_commands.hpp"

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

chassis::ChassisSpin2WinSubsystem theChassis(drivers());

turret::TurretSubsystem theTurret(drivers(), &yawMotor);
feeder::FeederPositionSubsystem theFeeder(drivers());
flywheel::FlywheelSubsystem theFlywheel(drivers());

/* define commands ----------------------------------------------------------*/
/* chassis */
chassis::ChassisRelativeDriveCommand chassisRelativeDrive(&theChassis, drivers(), &yawMotor);
chassis::ChassisSpin2winCommand chassisSpinDrive(&theChassis, drivers(), &yawMotor);
chassis::ChassisSpin2winKeyboardCommand chassisKeyboardDrive(&theChassis, drivers(), &yawMotor);
// chassis::ChassisCalibrateImuCommand chassisImuCalibrate(&theChassis, drivers());

/* turret */
turret::TurretManualAimCommand turretManualNoSpin(&theTurret, drivers());
turret::TurretMouseAimCommand turretMouseNoSpin(&theTurret, drivers());
turret::TurretStableManualAimCommand turretManualAim(&theTurret, &chassisSpinDrive, drivers());
turret::TurretStableMouseAimCommand turretMouseAim(&theTurret, &chassisKeyboardDrive, drivers());

// turret::TurretTestBottomLeftCommand turretLeftAim(&theTurret, drivers()); // Used for tuning
// turret::TurretTestTopRightCommand turretRightAim(&theTurret, drivers()); // Used for tuning

/* feeder */
feeder::FeederMoveUnjamCommand feederMoveUnjam(&theFeeder, drivers());

/* flywheel */
flywheel::FlywheelFireCommand flywheelStart(&theFlywheel, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/
/* Controller mappings states */
RemoteMapState feedFeederState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
RemoteMapState startFlywheelState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP);
RemoteMapState toggleChassisSpinState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);

// /* Controller mappings */
// HoldRepeatCommandMapping feedFeeder(drivers(), {&feederMoveUnjam}, &feedFeederState, true);
// HoldCommandMapping startFlywheel(drivers(), {&flywheelStart, &feederMoveUnjam}, &startFlywheelState);
// HoldCommandMapping toggleChassisSpin(drivers(), {&chassisSpinDrive, &turretManualAim}, &toggleChassisSpinState);

// /* Mouse mappings states */
RemoteMapState mouseStartFlywheelState(RemoteMapState::MouseButton::RIGHT);
RemoteMapState mouseFeedFeederState(RemoteMapState::MouseButton::LEFT);
RemoteMapState toggleChassisSpinKeyState({Remote::Key::R});

/* Controller mappings */
HoldRepeatCommandMapping* feedFeeder = nullptr;
HoldCommandMapping* startFlywheel = nullptr;
HoldCommandMapping* toggleChassisSpin = nullptr;
ToggleCommandMapping* mouseStartFlywheel = nullptr;
HoldRepeatCommandMapping* mouseFeedFeeder = nullptr;
ToggleCommandMapping* toggleChassisSpinKey = nullptr;

/* Mouse mappings */
// ToggleCommandMapping mouseStartFlywheel(drivers(), {&flywheelStart}, &mouseStartFlywheelState);
// HoldRepeatCommandMapping mouseFeedFeeder(drivers(), {&feederMoveUnjam}, &mouseFeedFeederState, true);
// ToggleCommandMapping toggleChassisSpinKey(drivers(), {&chassisKeyboardDrive, &turretMouseAim}, &toggleChassisSpinKeyState);


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
    theTurret.setDefaultCommand(&turretManualNoSpin);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *) {
    // drivers->commandScheduler.addCommand(&chassisImuCalibrate);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(std::unique_ptr<tap::control::CommandMapping>(feedFeeder));
    drivers->commandMapper.addMap(std::unique_ptr<tap::control::CommandMapping>(startFlywheel));
    drivers->commandMapper.addMap(std::unique_ptr<tap::control::CommandMapping>(toggleChassisSpin));
    drivers->commandMapper.addMap(std::unique_ptr<tap::control::CommandMapping>(mouseStartFlywheel));
    drivers->commandMapper.addMap(std::unique_ptr<tap::control::CommandMapping>(mouseFeedFeeder));
    drivers->commandMapper.addMap(std::unique_ptr<tap::control::CommandMapping>(toggleChassisSpinKey));
}

void initSubsystemCommands(src::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    
    feedFeeder = new HoldRepeatCommandMapping(drivers, {&feederMoveUnjam}, &feedFeederState, true);
    startFlywheel = new HoldCommandMapping(drivers, {&flywheelStart, &feederMoveUnjam}, &startFlywheelState);
    toggleChassisSpin = new HoldCommandMapping(drivers, {&chassisSpinDrive, &turretManualAim}, &toggleChassisSpinState);
    mouseStartFlywheel = new ToggleCommandMapping(drivers, {&flywheelStart}, &mouseStartFlywheelState);
    mouseFeedFeeder = new HoldRepeatCommandMapping(drivers, {&feederMoveUnjam}, &mouseFeedFeederState, true);
    toggleChassisSpinKey = new ToggleCommandMapping(drivers, {&chassisKeyboardDrive, &turretMouseAim}, &toggleChassisSpinKeyState);

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

#endif  // TARGET_SPIN_TO_WIN
