#ifndef CV_SERIAL_HPP_
#define CV_SERIAL_HPP_

#include <cstdint>

#include "tap/communication/serial/uart.hpp"
#include "tap/util_macros.hpp"

#include "communication/cv_serial_data.hpp"

using tap::Drivers;
using tap::communication::serial::Uart;

namespace src::communication::cv
{
/**
 * A serial handler that implements a specific protocol to be used for
 * communicating with the computer vision Jetson.
 *
 * Extend this class and implement messageReceiveCallback if you
 * want to use this serial protocol on a serial line.
 *
 * Structure of a Serial Message:
 * \rst
 * +-----------------+------------------------------------------------------------+
 * | Byte Number     | Byte Description                                           |
 * +=================+============================================================+
 * | Frame Header                                                                 |
 * +-----------------+------------------------------------------------------------+
 * | 0               | Start of file byte                                         |
 * +-----------------+------------------------------------------------------------+
 * | 1               | Message Type                                               |
 * +-----------------+------------------------------------------------------------+
 * | 2               | Message Data Length                                        |
 * +-----------------+------------------------------------------------------------+
 * +-----------------+------------------------------------------------------------+
 * | Body - Data Length bytes                                                     |
 * +-----------------+------------------------------------------------------------+
 * +-----------------+------------------------------------------------------------+
 * \endrst
 */
class CVSerial : public CVSerialData
{
public:
    /**
     * The serial message's frame header.
     */
    struct FrameHeader
    {
        uint8_t startOfFrame;
        uint8_t messageID;
        uint8_t dataLength;
    } modm_packed;

    /**
     * A container for storing and sending message over serial.
     */
    template <int DATA_SIZE>
    struct SerialMessage
    {
        /**
         * Constructs a SerialMessage. In doing so this constructor configures the message header.
         *
         * @param[in] messageID Message ID byte.
         */
        explicit SerialMessage(uint8_t messageID = 0)
        {
            header.startOfFrame = 0xFC;
            header.messageID = messageID;
            header.dataLength = static_cast<uint8_t>(sizeof(data));
        }

        FrameHeader header;
        uint8_t data[DATA_SIZE];
    } modm_packed;

    static const uint16_t SERIAL_RX_BUFF_SIZE = 256;
    static const uint16_t SERIAL_HEAD_BYTE = 0xFC;

    using ReceivedSerialMessage = SerialMessage<SERIAL_RX_BUFF_SIZE>;

    /**
     * Construct a Serial object.
     *
     * @param[in] port serial port to work on.
     */
    CVSerial(Drivers *drivers, Uart::UartPort port);
    DISALLOW_COPY_AND_ASSIGN(CVSerial)
    mockable ~CVSerial() = default;

    /**
     * Initialize serial. In particular, initializes the hardware serial
     * specified upon construction.
     *
     * @see `Uart`
     */
    mockable void initialize();

    /**
     * Receive messages. Call periodically in order to receive all
     * incoming messages.
     *
     * @note tested with a delay of 10 microseconds with referee system. The
     *      longer the timeout the more likely a message failure may occur.
     */
    mockable void updateSerial();

    /**
     * Called when a complete message is received. A derived class must
     * implement this in order to handle incoming messages properly.
     *
     * @param[in] completeMessage a reference to the full message that has
     *      just been received by this class.
     */
    virtual void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) = 0;

    Uart::UartPort getUartPort() const { return port; } ;

private:
    enum SerialRxState
    {
        SERIAL_HEADER_SEARCH,  /// A header byte has not yet been received.
        PROCESS_FRAME_HEADER,  /// A header is received and the frame header is being processed.
        PROCESS_FRAME_DATA     /// The data is being processed.
    };
    
    /// The serial port you are connected to.
    Uart::UartPort port;

    /// stuff for RX, buffers to store parts of the header, state machine.
    SerialRxState cvSerialRxState;

    /// Message in middle of being constructed.
    ReceivedSerialMessage newMessage;

    /// Most recent complete message.
    ReceivedSerialMessage mostRecentMessage;

    /**
     * The current zero indexed byte that is being read from the `Uart` class. An absolute index
     * into the `newMessage` object. `newMessage` reinterpreted as a uint8_t array and elements read
     * from serial are placed into the message.
     */
    uint16_t frameCurrReadByte;

protected:
    Drivers *drivers;
};

}  // namespace src::communication::cv

#endif  // CV_SERIAL_HPP_
