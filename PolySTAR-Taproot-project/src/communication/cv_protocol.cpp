#include "communication/cv_protocol.hpp"

namespace src 
{
namespace communication
{
namespace cv 
{
namespace Tx
{
    bool sendCVMessage(cvMessage message, tap::Drivers *drivers) {
        if (drivers->uart.isWriteFinished(Uart::UartPort::Uart7)) {
            drivers->uart.write(Uart::UartPort::Uart7,(uint8_t*)&message, DATA_LEN_PREFIX + message.dataLength);
            return true;
        }

        return false;
    } ;
}  // Namespace Tx
}  // Namespace cv
}  // Namespace communication
}  // Namespace src