#include "cv_handler.hpp"

#include "tap/architecture/endianness_wrappers.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap::arch;

namespace src::communication::cv
{
CVHandler::CVHandler(Drivers* drivers)
    : CVSerial(drivers, tap::communication::serial::Uart::UartPort::Uart7),
      turretData(),
      movementData(),
      shootOrderFlag(false)
{
}

void CVHandler::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    // char buffer[50];
    // int nBytes;
    switch (completeMessage.header.messageID)
    {
        case CVSerialData::Rx::TURRET_MESSAGE:
        {
            decodeToTurretData(completeMessage);
            // nBytes = sprintf(buffer,"Turret Message Received\n");
            break;
        }
        case CVSerialData::Rx::MOVEMENT_MESSAGE:
        {
            decodeToMovementData(completeMessage);
            // nBytes = sprintf(buffer,"Movement Message Received\n");
            break;
        }
        case CVSerialData::Rx::SHOOT_MESSAGE:
        {
            decodeToShootOrder();
            // nBytes = sprintf(buffer,"Shoot Message Received\n");
            break;
        }
        default:
            // nBytes = sprintf(buffer,"Message Not Recognized\n");
            break;
    }
    // drivers->uart.write(tap::communication::serial::Uart::Uart6,(uint8_t*)buffer,nBytes+1);
}

/*
* Decode turret setpoint data
*/
bool CVHandler::decodeToTurretData(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != sizeof(Rx::TurretData))
    {
        return false;
    }
    convertFromLittleEndian(&turretData.pitchSetpoint, message.data);
    convertFromLittleEndian(&turretData.yawSetpoint, message.data + 2);
    return true;
}

/*
* Decode chassis setpoint data
*/
bool CVHandler::decodeToMovementData(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != sizeof(Rx::MovementData))
    {
        return false;
    }
    convertFromLittleEndian(&movementData.xSetpoint, message.data);
    convertFromLittleEndian(&movementData.ySetpoint, message.data + 2);
    convertFromLittleEndian(&movementData.rSetpoint, message.data + 4);
    return true;
}

bool CVHandler::decodeToShootOrder()
{
    shootOrderFlag = true;
    return true;
}

/*
* Send CV Message over UART.
* Returns true if message was sent succesfully
*/
bool CVHandler::sendCVMessage(CVSerialData::Tx::CVMessageHeader &message) {
    // TODO waiting for the buffer to empty every time slows us down. It's likely the buffer is big enough that we won't have 
    // to worry about overflowing it, but needs to be tested
    if (drivers->uart.isWriteFinished(getUartPort())) {
        drivers->uart.write(getUartPort(), reinterpret_cast<uint8_t*>(&message), CVSerialData::Tx::DATA_LEN_PREFIX + message.dataLength);
        return true;
    }
    return false;
} ;

}  // namespace src::communication::cv
