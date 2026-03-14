#include "default_formatter.hpp"

#include <cstdio>

namespace polylog
{

DefaultFormatter::~DefaultFormatter() = default;

void DefaultFormatter::format(const LogMessage& log, char* buffer, size_t size)
{
    constexpr const char* format = "[%010lu] [%s] %s: %s";
    if (size == 0 || buffer == nullptr) return;

    int written = std::snprintf(
        buffer,
        size,
        format,
        log.timestamp_ms,
        logLevelToStringView(log.level).data(),
        log.loggerName.data(),
        log.payload.data());

    if (written < 0 || static_cast<size_t>(written) >= size)
    {
        buffer[size - 1] = '\0';
    }
}

}  // namespace polylog