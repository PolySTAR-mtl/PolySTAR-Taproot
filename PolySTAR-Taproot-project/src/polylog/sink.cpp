#include "sink.hpp"

namespace polylog
{

Sink::Sink(Formatter* formatter) : formatter_{formatter}, buffer_{} {}

Sink::~Sink() = default;

void Sink::log(const LogMessage& log)
{
    formatter_->format(log, buffer_, BUFFER_SIZE);
    write(buffer_);
    buffer_[0] = '\0';
}

}  // namespace polylog