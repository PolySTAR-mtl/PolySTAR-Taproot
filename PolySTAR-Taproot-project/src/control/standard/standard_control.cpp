#ifdef TARGET_STANDARD

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "control/drivers/drivers_singleton.hpp"
#include "control/drivers/drivers.hpp"
#include "control/safe_disconnect.hpp"

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

/* define commands ----------------------------------------------------------*/

/* safe disconnect function -------------------------------------------------*/
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* define command mappings --------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(src::Drivers *drivers) {}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(src::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(src::Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(src::Drivers *drivers) {} // should be empty

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
