#ifndef LOG_IMPL_HPP
#define LOG_IMPL_HPP

#include "log.hpp"

namespace polylog
{


template <typename... Args>
void trace(const char* format, Args&&... args) {
    defaultLogger().trace(format, std::forward<Args>(args)...);
}

template <typename... Args>
void debug(const char* format, Args&&... args) {
    defaultLogger().debug(format, std::forward<Args>(args)...);
}

template <typename... Args>
void info(const char* format, Args&&... args) {
    defaultLogger().info(format, std::forward<Args>(args)...);
}

template <typename... Args>
void warning(const char* format, Args&&... args) {
    defaultLogger().warning(format, std::forward<Args>(args)...);
}

template <typename... Args>
void error(const char* format, Args&&... args) {
    defaultLogger().error(format, std::forward<Args>(args)...);
}

template <typename... Args>
void critical(const char* format, Args&&... args) {
    defaultLogger().critical(format, std::forward<Args>(args)...);
}

}

#endif