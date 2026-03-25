#ifndef DEFAULT_FORMATTER_HPP
#define DEFAULT_FORMATTER_HPP

#include "formatter.hpp"

namespace polylog
{

struct DefaultFormatter : public Formatter
{
    ~DefaultFormatter() override;

    [[nodiscard]] size_t format(const LogMessage& log, std::span<char> buffer) override;
};

}  // namespace polylog

#endif  // DEFAULT_FORMATTER_HPP