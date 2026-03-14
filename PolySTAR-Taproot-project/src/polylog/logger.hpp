#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <array>
#include <string_view>

#include "log_level.hpp"
#include "log_message.hpp"
#include "sink.hpp"

namespace polylog
{

template <LogLevel Level, size_t MaxSinks = 5>
class Logger
{
public:
    explicit Logger(const char* name);

    bool addSink(Sink* sink);

    template <LogLevel MessageLevel>
    void log(std::string_view message);

private:
    size_t sinkCount_;
    std::array<Sink*, MaxSinks> sinks_;
    const char* name_;
};

}  // namespace polylog

#include "logger_impl.hpp"

#endif  // LOGGER_HPP