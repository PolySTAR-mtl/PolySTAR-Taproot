#include "default_formatter.hpp"

#include <cstdio>

#include <span>

namespace polylog
{

DefaultFormatter::~DefaultFormatter() = default;

[[nodiscard]] size_t DefaultFormatter::format(const LogMessage& log, std::span<char> buffer)
{
    constexpr const char* formatString = "[%010lu] [%.*s] %.*s: %.*s";
    const size_t size = buffer.size();

    if (buffer.empty()) return 0;

    const std::string_view severity = severityToStringView(log.severity);
    const std::string_view name = log.loggerName;
    const std::string_view payload = log.payload;

    int written = std::snprintf(
        buffer.data(),
        size,
        formatString,
        log.timestamp_ms,
        static_cast<int>(severity.size()),
        severity.data(),
        static_cast<int>(name.size()),
        name.data(),
        static_cast<int>(payload.size()),
        payload.data());

    if (written < 0)
    {
        buffer[0] = '\0';
        return 0;
    }

    if (static_cast<size_t>(written) >= size)
    {
        buffer[size - 1] = '\0';
        return size - 1;
    }
    return static_cast<size_t>(written);
}

}  // namespace polylog