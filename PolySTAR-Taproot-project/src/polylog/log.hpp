#ifndef LOG_HPP
#define LOG_HPP

#include <string_view>

#include "tap/communication/serial/uart.hpp"

#include "log_level.hpp"

namespace polylog
{

constexpr LogLevel LOG_LEVEL = LogLevel::Info;

void log_init(tap::communication::serial::Uart* uart);

void trace(std::string_view message);

void debug(std::string_view message);

void info(std::string_view message);

void warning(std::string_view message);

void error(std::string_view message);

void critical(std::string_view message);

}  // namespace polylog

#endif  // LOG_HPP