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
    switch (completeMessage.header.messageID)
    {
        case CVSerialData::Rx::TURRET_MESSAGE:
        {
            decodeToTurretData(completeMessage);
            break;
        }
        case CVSerialData::Rx::MOVEMENT_MESSAGE:
        {
            decodeToMovementData(completeMessage);
            break;
        }
        case CVSerialData::Rx::SHOOT_MESSAGE:
        {
            decodeToShootOrder();
            break;
        }
        default:
            break;
    }
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
bool CVHandler::sendCVMessage(CVSerialData::Tx::CVMessageHeader message) {
    if (drivers->uart.isWriteFinished(Uart::UartPort::Uart7)) {
        drivers->uart.write(Uart::UartPort::Uart7,(uint8_t*)&message, CVSerialData::Tx::DATA_LEN_PREFIX + message.dataLength);
        return true;
    }
    return false;
} ;

}  // namespace src::communication::cv
