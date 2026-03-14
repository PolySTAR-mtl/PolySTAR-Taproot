#ifndef LOGGER_IMPL_HPP
#define LOGGER_IMPL_HPP

#include "log_level.hpp"
#include "logger.hpp"

#include "tap/architecture/clock.hpp"

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
template <LogLevel MessageLevel>
void Logger<Level, MaxSinks>::log(std::string_view message)
{
    if constexpr (MessageLevel < Level)
    {
        return;
    }

    LogMessage log{
        .level = MessageLevel,
        .loggerName = name_,
        .payload = message,
        .timestamp_ms = tap::arch::clock::getTimeMilliseconds() };

    for (const auto& sink : sinks_)
    {
        sink->log(log);
    }
}

}  // namespace polylog

#endif  // LOGGER_IMPL_HPP