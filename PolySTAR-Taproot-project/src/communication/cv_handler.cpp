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
      shootOrderData()
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
            decodeToShootOrder(completeMessage);
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

bool CVHandler::decodeToShootOrder(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != sizeof(Rx::ShootOrderData))
    {
        return false;
    }
    shootOrderData.shootOrder = static_cast<uint8_t>(message.data[0]);
    return true;
}

void CVHandler::processGameStage()
{
    RefSerialData::Rx::GameData gameData = drivers->refSerial.getGameData();
    if (gameData.gameStage != lastGameStage)
    {
        CVSerialData::Tx::EventMessage eventMessage;
        eventMessage.gameStage = gameData.gameStage;
        drivers->uart.write(Uart::UartPort::Uart7, (uint8_t *)(&eventMessage), sizeof(eventMessage));
        lastGameStage = gameData.gameStage;
    }
}

}  // namespace src::communication::cv
