#ifndef SINK_HPP
#define SINK_HPP

#include <string_view>

#include "formatter.hpp"
#include "log_message.hpp"

namespace polylog
{

class Sink
{
public:
    explicit Sink(Formatter* formatter);

    virtual ~Sink();

    void log(const LogMessage& log);

protected:
    virtual void write(std::string_view text) = 0;
    virtual void flush() = 0;

public:
    static constexpr size_t BUFFER_SIZE = 256;
    
private:
    Formatter* formatter_;
    std::array<char, BUFFER_SIZE> buffer_;
};

}  // namespace polylog

#endif  // SINK_HPP