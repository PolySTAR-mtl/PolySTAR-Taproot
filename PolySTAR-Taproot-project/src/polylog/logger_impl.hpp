#ifndef LOGGER_IMPL_HPP
#define LOGGER_IMPL_HPP

#include "tap/architecture/clock.hpp"

#include "log_level.hpp"
#include "logger.hpp"

namespace polylog
{

template <LogLevel Level, size_t MaxSinks>
Logger<Level, MaxSinks>::Logger(const char* name) : sinkCount_{},
                                                    sinks_{},
                                                    name_{name}
{
}

template <LogLevel Level, size_t MaxSinks>
bool Logger<Level, MaxSinks>::addSink(Sink* sink)
{
    if (sink == nullptr || sinkCount_ >= MaxSinks)
    {
        return false;
    }
    sinks_[sinkCount_++] = sink;
    return true;
}

template <LogLevel Level, size_t MaxSinks>
template <LogLevel MessageLevel, typename... Args>
void Logger<Level, MaxSinks>::log(const char* format, Args&&... args)
{
    if constexpr (MessageLevel < Level)
    {
        return;
    }

    std::array<char, Sink::BUFFER_SIZE> buffer{};
    int written = std::snprintf(buffer.data(), buffer.size(), format, std::forward<Args>(args)...);

    const auto getLength = [](int written) -> size_t
    {
        if (written < 0) return 0;

        if (static_cast<size_t>(written) >= buffer.size()) return buffer.size() - 1;

        return static_cast<size_t>(written);
    };

    size_t length = getLength(written);
    buffer[length] = '\0';

    std::string_view message{buffer, length};

    LogMessage log{
        .level = MessageLevel,
        .loggerName = name_,
        .payload = message,
        .timestamp_ms = tap::arch::clock::getTimeMilliseconds()};

    for (const auto& sink : sinks_)
    {
        sink->log(log);
    }
}

}  // namespace polylog

#endif  // LOGGER_IMPL_HPP