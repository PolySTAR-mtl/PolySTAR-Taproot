#ifndef CV_HANDLER_HPP_
#define CV_HANDLER_HPP_

#include <cstdint>
#include <unordered_map>

#include "tap/architecture/timeout.hpp"
#include "tap/util_macros.hpp"

#include "modm/container/deque.hpp"
#include "modm/processing/protothread/semaphore.hpp"

#include "cv_serial.hpp"
#include "cv_serial_data.hpp"

namespace src
{
class Drivers;
}

namespace src::communication::cv
{
/**
 * A class designed to communicate with the 2023 version of the CV Jetson.
 * Supports decoding various CV message types. Also supports sending
 * feedback messages to the Jetson.
 *
 * @note use the instance stored in the `Drivers` to interact with this class
 *      (you shouldn't be declaring your own `CVHandler` object).
 *
 * Receive information from the referee serial by continuously calling `messageReceiveCallback`.
 * Access data sent by the referee serial by calling `getMovementData`, `getTurretData` or `getShootOrder`.
 */
class CVHandler : public CVSerial
{
public:
    /**
     * Constructs a CVHandler class connected to UART Port 7 with
     * CRC enforcement disabled.
     *
     * @see `CVSerial` and `DJISerial`
     */
    CVHandler(Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(CVHandler)
    mockable ~CVHandler() = default;

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;
    
    /**
     * Checks if the game stage has changed and sends the new GameStage to CV if necessary.
    */
    void processGameStage();

    /**
     * Returns a reference to the most up to date turret setpoint struct.
     */
    mockable const Rx::TurretData& getTurretData() const { return turretData; };

    /**
     * Returns a reference to the most up to date movement setpoint struct.
     */
    mockable const Rx::MovementData& getMovementData() const { return movementData; };

     /**
     * Returns a reference to the most up to date shoot order struct.
     */
    mockable const Rx::ShootOrderData& getShootOrderData() const { return shootOrderData; };

private:
    Rx::TurretData turretData;
    Rx::MovementData movementData;
    Rx::ShootOrderData shootOrderData;
    RefSerialData::Rx::GameStage lastGameStage = RefSerialData::Rx::GameStage::END_GAME;
    /**
     * Decodes CV serial message containing turret yaw and pitch setpoints
     */
    bool decodeToTurretData(const ReceivedSerialMessage& message);
    /**
     * Decodes CV serial message containing the chassis velocity setpoints.
     */
    bool decodeToMovementData(const ReceivedSerialMessage& message);
    /**
     * Decodes CV serial message containing order to fire
     */
    bool decodeToShootOrder(const ReceivedSerialMessage& message);
};

}  // namespace src::communication::cv

#endif  // CV_HANDLER_HPP_
