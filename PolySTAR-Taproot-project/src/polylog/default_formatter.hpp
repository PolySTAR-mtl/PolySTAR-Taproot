#ifndef DEFAULT_FORMATTER_HPP
#define DEFAULT_FORMATTER_HPP

#include "formatter.hpp"

namespace polylog
{

struct DefaultFormatter : public Formatter
{
    ~DefaultFormatter() override;

    void format(const LogMessage& log, char* buffer, size_t size) override;
};

}  // namespace polylog

#endif  // DEFAULT_FORMATTER_HPP