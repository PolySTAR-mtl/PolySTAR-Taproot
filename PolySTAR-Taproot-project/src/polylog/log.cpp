#include "log.hpp"

#include "default_formatter.hpp"
#include "logger.hpp"
#include "serial_sink.hpp"

namespace polylog
{

void Log::init(tap::communication::serial::Uart* uart)
{
    static DefaultFormatter defaultFormatter{};
    static SerialSink<tap::communication::serial::Uart::UartPort::Uart8> serialSink{
        &defaultFormatter,
        nullptr};

    if (uart == nullptr) return;

    serialSink.setUart(uart);
    getGlobalLogger().addSink(&serialSink);
}

Log::GlobalLogger& Log::getGlobalLogger()
{
    static Logger<GLOBAL_LOG_LEVEL> globalLogger{"Global"};
    return globalLogger;
}

}  // namespace polylog