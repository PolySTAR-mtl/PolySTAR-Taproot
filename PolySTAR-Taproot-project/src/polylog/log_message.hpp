#ifndef LOG_MESSAGE_HPP
#define LOG_MESSAGE_HPP

#include <string_view>

#include "log_level.hpp"

namespace polylog
{

struct LogMessage
{
    LogLevel level;
    std::string_view loggerName;
    std::string_view payload;
    uint32_t timestamp_ms;
};

}  // namespace polylog

#endif  // LOG_MESSAGE_HPP