#ifndef CV_SERIAL_DATA_HPP_
#define CV_SERIAL_DATA_HPP_

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/util_macros.hpp"

using tap::communication::serial::RefSerialData;
using tap::communication::serial::Uart;

namespace src::communication::cv
{
/**
 * Contains enum and struct definitions used in the `CVSerial` and `CVHandler` classes
 * 
 */
class CVSerialData
{
public:
    class Tx
    {
    public:
        // Outgoing message IDs
        enum MESSAGE_ID : uint8_t {
            GAME_STATUS_MESSAGE = 0x01,
            EVENT_MESSAGE = 0x02,
            TURRET_FEEDBACK_MESSAGE = 0x03,
            POSITION_FEEDBACK_MESSAGE = 0x04
        };

        // Message lengths in bytes
        enum DATA_LEN : uint8_t {
            DATA_LEN_GAME = 14,
            DATA_LEN_EVENT = 2,
            DATA_LEN_TURRET = 4,
            DATA_LEN_POSITION = 22,
            DATA_LEN_PREFIX = 3
        };

        // Aiming mode IDs
        enum AIMING_MODE : uint8_t {
            MANUAL_AIM,
            AUTO_AIM,
            AUTO_FIRE,
            FULL_AUTO
        };

        // Start of frame byte
        static const uint8_t START_OF_FRAME = 0xFC;

        struct CVMessageHeader {
            CVMessageHeader(uint8_t sof, uint8_t commandID, uint8_t dataLength) : startOfFrame(sof), commandID(commandID), dataLength(dataLength) {};
            uint8_t startOfFrame;;
            uint8_t commandID;
            uint8_t dataLength;
        } modm_packed;

        struct GameStatusMessage : CVMessageHeader {
            GameStatusMessage() : CVMessageHeader(START_OF_FRAME, GAME_STATUS_MESSAGE, DATA_LEN_GAME) {};
            uint16_t robotType;
            uint16_t redStdHP;
            uint16_t redHroHP;
            uint16_t redStyHP;
            uint16_t blueStdHP;
            uint16_t blueHroHP;
            uint16_t blueStyHP;
            uint16_t aimingMode;
        } modm_packed;

        struct EventMessage : CVMessageHeader {
            EventMessage() : CVMessageHeader(START_OF_FRAME, EVENT_MESSAGE, DATA_LEN_EVENT) {};
            RefSerialData::Rx::GameStage gameStage;
        } modm_packed;

        struct TurretMessage : CVMessageHeader {
            TurretMessage() : CVMessageHeader(START_OF_FRAME, TURRET_FEEDBACK_MESSAGE, DATA_LEN_TURRET) {};
            int16_t pitch;
            int16_t yaw;
        } modm_packed;

        struct PositionMessage : CVMessageHeader {
            PositionMessage() : CVMessageHeader(START_OF_FRAME, POSITION_FEEDBACK_MESSAGE, DATA_LEN_POSITION) {};
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
        } modm_packed;

    };  // Class Tx

    class Rx 
    {
    public:
        // Incoming message IDs
        enum MESSAGE_ID : uint8_t {
            TURRET_MESSAGE = 0x10,
            MOVEMENT_MESSAGE = 0x11,
            SHOOT_MESSAGE = 0x12
        };

        /*
        * Turret pitch and yaw setpoints as received from CV
        * Angles given in millirad in the local body frame (Origin is barrel straight ahead, parallel to ground)
        */
        struct TurretData {
            int16_t pitchSetpoint;
            int16_t yawSetpoint;
        };

        /*
        * Chassis movement setpoints as received from CV
        * x, y in mm/s
        * r in millirad/s
        */
        struct MovementData {
            int16_t xSetpoint;
            int16_t ySetpoint;
            int16_t rSetpoint;
        };

    };  // Class Rx
}; // Class CVSerialData
}  // Namespace src::communication::cv

#endif // CV_SERIAL_DATA_HPP_