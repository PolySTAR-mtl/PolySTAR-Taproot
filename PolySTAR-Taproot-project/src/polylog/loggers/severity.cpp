#include "severity.hpp"

namespace polylog
{

[[nodiscard]] std::string_view severityToStringView(Severity severity)
{
    switch (severity)
    {
        case Severity::Trace:
            return "Trace";

        case Severity::Debug:
            return "Debug";

        case Severity::Info:
            return "Info";

        case Severity::Warning:
            return "Warning";

        case Severity::Error:
            return "Error";

        case Severity::Critical:
            return "Critical";
        
        default:
            return "Unknown";
    }
}

}  // namespace polylog