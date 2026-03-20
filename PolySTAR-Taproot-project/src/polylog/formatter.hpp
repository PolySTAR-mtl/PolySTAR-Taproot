#ifndef FORMATTER_HPP
#define FORMATTER_HPP

#include <span>

#include "log_message.hpp"

namespace polylog
{

struct Formatter
{
    virtual ~Formatter();

    [[nodiscard]] virtual size_t format(const LogMessage& log, std::span<char> buffer) = 0;
};

}  // namespace polylog

#endif  // FORMATTER_HPP