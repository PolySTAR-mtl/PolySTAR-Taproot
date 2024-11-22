#ifdef TARGET_CV_TESTBENCH

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/drivers/drivers.hpp"
#include "control/drivers/drivers_singleton.hpp"
#include "control/safe_disconnect.hpp"

// Turret includes
#include "subsystems/turret/turret_auto_aim_command.hpp"
#include "subsystems/turret/turret_test_bottomleft_command.hpp"
#include "subsystems/turret/turret_manual_aim_command.hpp"
#include "subsystems/turret/turret_mouse_aim_command.hpp"
#include "subsystems/turret/turret_test_topright_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_test_auto_aim_command.hpp"

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

turret::TurretSubsystem theTurret(drivers());


/* define commands ----------------------------------------------------------*/

/* turret -------------------------------------------------------------------*/
turret::TurretManualAimCommand turretManualAim(&theTurret, drivers());
turret::TurretTestBottomLeftCommand turretLeftAim(&theTurret, drivers());
turret::TurretTestTopRightCommand turretRightAim(&theTurret, drivers());
turret::TurretMouseAimCommand turretMouseAim(&theTurret, drivers());
turret::TurretAutoAimCommand turretAutoAim(&theTurret, drivers());
turret::TurretTestAutoAimCommand turretTestAutoAim(&theTurret, drivers());

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/

// HoldCommandMapping startFlywheel(drivers(), {&flywheelStart}, RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
/*-Turret-*/
ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));

/*-Auto commands*/

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&theTurret);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    theTurret.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *)
{
    theTurret.setDefaultCommand(&turretAutoAim);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers)
{
    // Nothing to do
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers)
{
    /*-Ammo Booster-*/
    drivers->commandMapper.addMap(&turretMouseAimToggle);
    /*-Chassis-*/
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
    int nBytes = sprintf(buffer, "Initializing Testbench\n");
    drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart8,
        (uint8_t *)buffer,
        nBytes + 1);
}

}  // namespace control

#endif  // TARGET_SENTRY
