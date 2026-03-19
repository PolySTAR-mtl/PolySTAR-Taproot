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

public:
    static constexpr size_t BUFFER_SIZE = 256;
    
private:
    Formatter* formatter_;
    char buffer_[BUFFER_SIZE];
};

}  // namespace polylog

#endif  // SINK_HPP