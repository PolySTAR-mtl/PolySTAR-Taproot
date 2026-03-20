#include "sink.hpp"

namespace polylog
{

Sink::Sink(Formatter* formatter) : formatter_{formatter}, buffer_{} {}

Sink::~Sink() = default;

void Sink::log(const LogMessage& log)
{
    if (formatter_ == nullptr) return;
    size_t length = formatter_->format(log, std::span{buffer_.data(), buffer_.size()});
    write(std::string_view{buffer_.data(), length});
}

}  // namespace polylog