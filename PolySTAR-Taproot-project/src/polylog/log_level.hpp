#ifndef LOG_LEVEL_HPP
#define LOG_LEVEL_HPP

#include <string_view>

namespace polylog
{

enum class LogLevel
{
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical
};

std::string_view logLevelToStringView(LogLevel level);

}  // namespace poly_log

#endif  // LOG_LEVEL_HPP