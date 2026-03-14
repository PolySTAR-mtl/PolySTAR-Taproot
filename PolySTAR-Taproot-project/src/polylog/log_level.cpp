#include "log_level.hpp"

namespace polylog
{

std::string_view logLevelToStringView(LogLevel level)
{
    switch (level)
    {
        case LogLevel::Trace:
            return "Trace";

        case LogLevel::Debug:
            return "Debug";

        case LogLevel::Info:
            return "Info";

        case LogLevel::Warning:
            return "Warning";

        case LogLevel::Error:
            return "Error";

        case LogLevel::Critical:
            return "Critical";
    }
}

}  // namespace polylog