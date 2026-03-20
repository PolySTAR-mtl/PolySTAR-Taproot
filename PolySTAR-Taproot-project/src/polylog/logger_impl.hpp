#ifndef LOGGER_IMPL_HPP
#define LOGGER_IMPL_HPP

#include <array>
#include <cstdio>
#include <string_view>

#include "tap/architecture/clock.hpp"

#include "log_level.hpp"
#include "logger.hpp"

namespace polylog
{

template <LogLevel Level, size_t MaxSinks>
constexpr Logger<Level, MaxSinks>::Logger(std::string_view name)
    : sinkCount_{},
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

    for (size_t i = 0; i < sinkCount_; ++i)
    {
        if (sinks_[i] == sink) return false;
    }

    sinks_[sinkCount_++] = sink;
    return true;
}

template <LogLevel Level, size_t MaxSinks>
template <typename... Args>
void Logger<Level, MaxSinks>::trace(const char* format, Args&&... args)
{
    log<LogLevel::Trace>(format, std::forward<Args>(args)...);
}

template <LogLevel Level, size_t MaxSinks>
template <typename... Args>
void Logger<Level, MaxSinks>::debug(const char* format, Args&&... args)
{
    log<LogLevel::Debug>(format, std::forward<Args>(args)...);
}

template <LogLevel Level, size_t MaxSinks>
template <typename... Args>
void Logger<Level, MaxSinks>::info(const char* format, Args&&... args)
{
    log<LogLevel::Info>(format, std::forward<Args>(args)...);
}

template <LogLevel Level, size_t MaxSinks>
template <typename... Args>
void Logger<Level, MaxSinks>::warning(const char* format, Args&&... args)
{
    log<LogLevel::Warning>(format, std::forward<Args>(args)...);
}

template <LogLevel Level, size_t MaxSinks>
template <typename... Args>
void Logger<Level, MaxSinks>::error(const char* format, Args&&... args)
{
    log<LogLevel::Error>(format, std::forward<Args>(args)...);
}

template <LogLevel Level, size_t MaxSinks>
template <typename... Args>
void Logger<Level, MaxSinks>::critical(const char* format, Args&&... args)
{
    log<LogLevel::Critical>(format, std::forward<Args>(args)...);
}

template <LogLevel Level, size_t MaxSinks>
template <LogLevel MessageLevel, typename... Args>
void Logger<Level, MaxSinks>::log(const char* format, Args&&... args)
{
    if constexpr (MessageLevel < Level)
    {
        return;
    }

    if (sinkCount_ == 0)
    {
        return;
    }

    std::array<char, Sink::BUFFER_SIZE> buffer{};
    int written = std::snprintf(buffer.data(), buffer.size(), format, std::forward<Args>(args)...);

    const auto getLength = [bufferSize = buffer.size()](int written) -> size_t
    {
        if (written < 0) return 0;

        if (static_cast<size_t>(written) >= bufferSize) return bufferSize - 1;

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

    for (size_t i = 0; i < sinkCount_; ++i)
    {
        if (sinks_[i] == nullptr) continue;

        sinks_[i]->log(log);
    }
}

}  // namespace polylog

#endif  // LOGGER_IMPL_HPP