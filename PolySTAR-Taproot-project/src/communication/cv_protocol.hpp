
#include "control/drivers/drivers.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

using tap::communication::serial::RefSerialData;

namespace src 
{
namespace communication
{
namespace cv 
{
    // Outgoing message IDs
    const uint8_t GAME_STATUS_MESSAGE = 0x01;
    const uint8_t EVENT_MESSAGE = 0x02;
    const uint8_t TURRET_FEEDBACK_MESSAGE = 0x03;
    const uint8_t POSITION_FEEDBACK_MESSAGE = 0x04;

    // Message lengths in bytes
    const uint8_t DATA_LEN_GAME = 14;
    const uint8_t DATA_LEN_EVENT = 2;
    const uint8_t DATA_LEN_TURRET = 4;
    const uint8_t DATA_LEN_POSITION = 22;
    const uint8_t DATA_LEN_PREFIX = 3;

    const uint8_t START_OF_FRAME = 0xFC;

    // Aiming mode IDs
    const uint8_t MANUAL_AIM = 14;
    const uint8_t AUTO_AIM = 2;
    const uint8_t AUTO_FIRE = 4;
    const uint8_t FULL_AUTO = 2;

    #pragma pack(push, 1)
    typedef struct GameStatusMessage {
        uint8_t sof = START_OF_FRAME;
        uint8_t commandID = GAME_STATUS_MESSAGE;
        uint8_t dataLength = DATA_LEN_GAME;
        uint8_t robotType;
        uint8_t redStdHP;
        uint8_t redHroHP;
        uint8_t redStyHP;
        uint8_t blueStdHP;
        uint8_t blueHroHP;
        uint8_t blueStyHP;
        uint8_t aimingMode;
    } GameStatusMessage;

    typedef struct EventMessage {
        uint8_t sof = START_OF_FRAME;
        uint8_t commandID = EVENT_MESSAGE;
        uint8_t dataLength = DATA_LEN_EVENT;
        RefSerialData::Rx::GameStage gameStage;
    } EventMessage;

    typedef struct TurretMessage {
        uint8_t sof = START_OF_FRAME;
        uint8_t commandID = TURRET_FEEDBACK_MESSAGE;
        uint8_t dataLength = DATA_LEN_TURRET;
        int16_t pitch;
        int16_t yaw;
    } TurretMessage;

    typedef struct PositionMessage {
        uint8_t paddingByte;
        uint8_t sof = START_OF_FRAME;
        uint8_t commandID = POSITION_FEEDBACK_MESSAGE;
        uint8_t dataLength = DATA_LEN_POSITION;
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
    } TurretMessage;
    #pragma pack(pop)

    bool sendCVMessage(PositionMessage message, tap::Drivers *drivers) {
        drivers->uart.write(Uart::UartPort::Uart7,(uint8_t*)&message, message.dataLength + DATA_LEN_PREFIX);
    }
}
}
}