#ifndef CV_PROTOCOL_HPP
#define CV_PROTOCOL_HPP


#include "control/drivers/drivers.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

using tap::communication::serial::RefSerialData;
using tap::communication::serial::Uart;

namespace src 
{
namespace communication
{
namespace cv 
{
namespace Tx
{
    // Outgoing message IDs
    enum MESSAGE_ID {
        NULL_MESSAGE,
        GAME_STATUS_MESSAGE,
        EVENT_MESSAGE,
        TURRET_FEEDBACK_MESSAGE,
        POSITION_FEEDBACK_MESSAGE
    };

    // Message lengths in bytes
    enum DATA_LEN {
        DATA_LEN_GAME = 14,
        DATA_LEN_EVENT = 2,
        DATA_LEN_TURRET = 4,
        DATA_LEN_POSITION = 22,
        DATA_LEN_PREFIX = 3
    };

    // Aiming mode IDs
    enum AIMING_MODE {
        MANUAL_AIM,
        AUTO_AIM,
        AUTO_FIRE,
        FULL_AUTO
    };

    // Start of frame byte
    const uint8_t START_OF_FRAME = 0xFC;

    #pragma pack(push, 1) // Pack structs to prevent padding causing issues when sending data
    struct cvMessage {
        cvMessage(uint8_t sof, uint8_t commandID, uint8_t dataLength) : startOfFrame(sof), commandID(commandID), dataLength(dataLength) {};
        uint8_t startOfFrame;;
        uint8_t commandID;
        uint8_t dataLength;
    };

    typedef struct GameStatusMessage : cvMessage {
        GameStatusMessage() : cvMessage(START_OF_FRAME, GAME_STATUS_MESSAGE, DATA_LEN_GAME) {};
        uint8_t robotType;
        uint8_t redStdHP;
        uint8_t redHroHP;
        uint8_t redStyHP;
        uint8_t blueStdHP;
        uint8_t blueHroHP;
        uint8_t blueStyHP;
        uint8_t aimingMode;
    } GameStatusMessage;

    typedef struct EventMessage : cvMessage {
        EventMessage() : cvMessage(START_OF_FRAME, EVENT_MESSAGE, DATA_LEN_EVENT) {};
        RefSerialData::Rx::GameStage gameStage;
    } EventMessage;

    typedef struct TurretMessage : cvMessage {
        TurretMessage() : cvMessage(START_OF_FRAME, TURRET_FEEDBACK_MESSAGE, DATA_LEN_TURRET) {};
        int16_t pitch;
        int16_t yaw;
    } TurretMessage;

    typedef struct PositionMessage : cvMessage {
        PositionMessage() : cvMessage(START_OF_FRAME, POSITION_FEEDBACK_MESSAGE, DATA_LEN_POSITION) {};
        int16_t Ax;
        int16_t Ay;
        int16_t Az;
        int16_t Gx;
        int16_t Gy;
        int16_t Gz;
        uint16_t frontLeftEncoder;
        uint16_t frontRightEncoder;
        uint16_t backLeftEncoder;
        uint16_t backRightEncoder;
        uint16_t dt;
    } PositionMessage;
    #pragma pack(pop) // Stop packing

    bool sendCVMessage(cvMessage message, tap::Drivers *drivers);

}  // Namespace Tx
}  // Namespace cv
}  // Namespace communication
}  // Namespace src

#endif