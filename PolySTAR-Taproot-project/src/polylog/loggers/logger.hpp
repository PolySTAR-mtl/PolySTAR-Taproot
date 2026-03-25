#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <array>
#include <string_view>

#include "log_message.hpp"
#include "severity.hpp"
#include "polylog/sinks/sink.hpp"

namespace polylog
{

template <Severity S, size_t MaxSinks = 5>
class Logger
{
public:
    constexpr Logger(std::string_view name);

    bool addSink(Sink* sink);

    template <typename... Args>
    void trace(const char* format, Args&&... args);

    template <typename... Args>
    void debug(const char* format, Args&&... args);

    template <typename... Args>
    void info(const char* format, Args&&... args);

    template <typename... Args>
    void warning(const char* format, Args&&... args);

    template <typename... Args>
    void error(const char* format, Args&&... args);

    template <typename... Args>
    void critical(const char* format, Args&&... args);

private:
    template <Severity MessageSeverity, typename... Args>
    void log(const char* format, Args&&... args);

private:
    size_t sinkCount_;
    std::array<Sink*, MaxSinks> sinks_;
    std::string_view name_;
};


}  // namespace polylog

#include "logger_impl.hpp"

#endif  // LOGGER_HPP