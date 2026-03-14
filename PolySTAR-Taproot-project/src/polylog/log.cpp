#include "log.hpp"

#include "default_formatter.hpp"
#include "logger.hpp"
#include "serial_sink.hpp"

namespace polylog
{

#ifdef LOG_LEVEL
constexpr LogLevel GLOBAL_LOG_LEVEL = LOG_LEVEL;
#else
constexpr LogLevel GLOBAL_LOG_LEVEL = LogLevel::Info;
#endif



static Logger<LOG_LEVEL_VALUE> global{"Global"};
static DefaultFormatter defaultFormatter{};

static SerialSink<tap::communication::serial::Uart::UartPort::Uart8> serialSink{
    &defaultFormatter,
    nullptr};

void log_init(tap::communication::serial::Uart* uart)
{
    if (uart == nullptr) return;

    serialSink.setUart(uart);
    global.addSink(&serialSink);
}

void trace(std::string_view message) { global.log<LogLevel::Trace>(message); }

void debug(std::string_view message) { global.log<LogLevel::Debug>(message); }

void info(std::string_view message) { global.log<LogLevel::Info>(message); }

void warning(std::string_view message) { global.log<LogLevel::Warning>(message); }

void error(std::string_view message) { global.log<LogLevel::Error>(message); }

void critical(std::string_view message) { global.log<LogLevel::Critical>(message); }

}  // namespace polylog