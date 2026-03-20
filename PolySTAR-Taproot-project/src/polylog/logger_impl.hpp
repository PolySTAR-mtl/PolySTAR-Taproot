#ifndef LOGGER_IMPL_HPP
#define LOGGER_IMPL_HPP

#include <array>
#include <cstdio>
#include <string_view>

#include "tap/architecture/clock.hpp"

#include "logger.hpp"
#include "severity.hpp"

namespace polylog
{

template <Severity S, size_t MaxSinks>
constexpr Logger<S, MaxSinks>::Logger(std::string_view name)
    : sinkCount_{},
      sinks_{},
      name_{name}
{
}

template <Severity S, size_t MaxSinks>
bool Logger<S, MaxSinks>::addSink(Sink* sink)
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

template <Severity S, size_t MaxSinks>
template <typename... Args>
void Logger<S, MaxSinks>::trace(const char* format, Args&&... args)
{
    log<Severity::Trace>(format, std::forward<Args>(args)...);
}

template <Severity S, size_t MaxSinks>
template <typename... Args>
void Logger<S, MaxSinks>::debug(const char* format, Args&&... args)
{
    log<Severity::Debug>(format, std::forward<Args>(args)...);
}

template <Severity S, size_t MaxSinks>
template <typename... Args>
void Logger<S, MaxSinks>::info(const char* format, Args&&... args)
{
    log<Severity::Info>(format, std::forward<Args>(args)...);
}

template <Severity S, size_t MaxSinks>
template <typename... Args>
void Logger<S, MaxSinks>::warning(const char* format, Args&&... args)
{
    log<Severity::Warning>(format, std::forward<Args>(args)...);
}

template <Severity S, size_t MaxSinks>
template <typename... Args>
void Logger<S, MaxSinks>::error(const char* format, Args&&... args)
{
    log<Severity::Error>(format, std::forward<Args>(args)...);
}

template <Severity S, size_t MaxSinks>
template <typename... Args>
void Logger<S, MaxSinks>::critical(const char* format, Args&&... args)
{
    log<Severity::Critical>(format, std::forward<Args>(args)...);
}

template <Severity S, size_t MaxSinks>
template <Severity MessageSeverity, typename... Args>
void Logger<S, MaxSinks>::log(const char* format, Args&&... args)
{
    if constexpr (MessageSeverity < S)
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
        .severity = MessageLevel,
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