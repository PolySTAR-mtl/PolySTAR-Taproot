#ifndef LOG_HPP
#define LOG_HPP

#include "logger.hpp"

namespace polylog
{

#ifdef LOG_LEVEL
using DefaultLogger = Logger<LOG_LEVEL>;
#else
using DefaultLogger = Logger<Severity::Info>;
#endif

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

DefaultLogger& defaultLogger();

}

#include "log_impl.hpp"

#endif