#include "default_formatter.hpp"

#include <cstdio>
#include <span>

namespace polylog
{

DefaultFormatter::~DefaultFormatter() = default;

[[nodiscard]] size_t DefaultFormatter::format(const LogMessage& log, std::span<char> buffer)
{
    constexpr const char* formatString = "[%010lu] [%s] %s: %s";
    const size_t size = buffer.size();

    if (buffer.empty()) return 0;

    int written = std::snprintf(
        buffer.data(),
        size,
        formatString,
        log.timestamp_ms,
        logLevelToStringView(log.level).data(),
        log.loggerName.data(),
        log.payload.data());

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