#ifndef FORMATTER_HPP
#define FORMATTER_HPP

#include "log_message.hpp"

namespace polylog
{

struct Formatter
{
    virtual ~Formatter();

    virtual void format(const LogMessage& log, char* buffer, size_t size) = 0;
};

}  // namespace polylog

#endif  // FORMATTER_HPP