#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "cv_serial.hpp"

/**
 * Macro that wraps uart read for ease of readability in code.
 * @param[in] data Byte array where the data should be read into.
 * @param[in] length The number of bytes to read.
 * @return The number of bytes read into data.
 */
#define READ(data, length) drivers->uart.read(this->port, data, length)

namespace src::communication::cv
{
CVSerial::CVSerial(Drivers *drivers, Uart::UartPort port)
    : port(port),
      cvSerialRxState(SERIAL_HEADER_SEARCH),
      newMessage(),
      mostRecentMessage(),
      frameCurrReadByte(0),
      drivers(drivers)
{
}

void CVSerial::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, 230400>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, 230400>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, 230400>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, 230400>();
            break;
        case Uart::UartPort::Uart7:
            drivers->uart.init<Uart::UartPort::Uart7, 230400>();
            break;
        case Uart::UartPort::Uart8:
            drivers->uart.init<Uart::UartPort::Uart8, 230400>();
            break;
        default:
            break;
    }
}

void CVSerial::updateSerial()
{
    switch (cvSerialRxState)
    {
        case SERIAL_HEADER_SEARCH:
        {
            // keep scanning for the head byte as long as you are here and have not yet found it.
            while (cvSerialRxState == SERIAL_HEADER_SEARCH && READ(&newMessage.header.startOfFrame, 1))
            {
                // we found it, store the head byte
                if (newMessage.header.startOfFrame == SERIAL_HEAD_BYTE)
                {
                    cvSerialRxState = PROCESS_FRAME_HEADER;
                    frameCurrReadByte = 0;
                }
            }
            break;
        }
        case PROCESS_FRAME_HEADER:  // the frame header consists of the type and length
        {
            frameCurrReadByte += READ(
                reinterpret_cast<uint8_t *>(&newMessage) + frameCurrReadByte + 1,
                sizeof(newMessage.header) - frameCurrReadByte - 1);

            // We have the complete message header in the frameHeader buffer
            if (frameCurrReadByte == sizeof(newMessage.header) - 1)
            {
                frameCurrReadByte = 0;

                if (newMessage.header.dataLength >= SERIAL_RX_BUFF_SIZE)
                {
                    cvSerialRxState = SERIAL_HEADER_SEARCH;
                    RAISE_ERROR(drivers, "received message length longer than allowed max");
                    return;
                }

                // move on to processing message body
                cvSerialRxState = PROCESS_FRAME_DATA;
            }
            break;
        }
        case PROCESS_FRAME_DATA:  // READ bulk of message
        {
            int bytesToRead = newMessage.header.dataLength;

            frameCurrReadByte += READ(
                reinterpret_cast<uint8_t *>(&newMessage) + sizeof(newMessage.header) +
                    frameCurrReadByte,
                bytesToRead - frameCurrReadByte);

            if (frameCurrReadByte == bytesToRead)
            {
                mostRecentMessage = newMessage;

                messageReceiveCallback(mostRecentMessage);

                cvSerialRxState = SERIAL_HEADER_SEARCH;
            }
            else if (frameCurrReadByte > bytesToRead)
            {
                frameCurrReadByte = 0;
                RAISE_ERROR(drivers, "Invalid message length");
                cvSerialRxState = SERIAL_HEADER_SEARCH;
            }
            break;
        }
    }
}

}  // namespace tap::communication::serial