#include "debug_subsystem.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;
using tap::communication::serial::RefSerialData;

namespace control
{
namespace debug
{
void DebugSubsystem::initialize()
{

}

void DebugSubsystem::refresh() {

    RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();

    // Skip sending debug messages if flag is disabled 
    // if (TURRET_DEBUG_MESSAGE == false) return;

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];

        // Robot level debug message
        int nBytes = sprintf (buffer, "Robot level: %i\n",
                              robotData.robotLevel);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        // Max barrel speed debug message
        nBytes = sprintf (buffer, "Max barrel speed: %i, ",
                              (int)(robotData.turret.barrelSpeedLimit17ID1*100));
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        // Last bullet speed debug message
        nBytes = sprintf (buffer, "Last bullet speed: %i\n",
                              (int)(robotData.turret.bulletSpeed*100));
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        // Chassis power debug message
        nBytes = sprintf (buffer, "Chassis max power: %i, Current chassis power: %i\n",
                              (int)(robotData.chassis.powerConsumptionLimit*100),
                              (int)(robotData.chassis.power*100));
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
    }
}

}  // namespace turret

}  // namespace control

