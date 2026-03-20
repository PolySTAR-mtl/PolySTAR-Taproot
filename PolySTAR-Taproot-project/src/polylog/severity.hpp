#ifndef SEVERITY_HPP
#define SEVERITY_HPP

#include <string_view>

namespace polylog
{

enum class Severity
{
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical
};

[[nodiscard]] std::string_view severityToStringView(Severity severity);

}  // namespace polylog

#endif  // SEVERITY_HPP