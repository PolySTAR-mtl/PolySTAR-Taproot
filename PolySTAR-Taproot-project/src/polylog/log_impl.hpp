#ifndef LOG_IMPL_HPP
#define LOG_IMPL_HPP

#include "log.hpp"

namespace polylog
{

template <typename... Args>
void Log::trace(const char* message, Args&&... args) {
    getGlobalLogger().log<LogLevel::Trace>(message, std::forward<Args>(args)...);
}

template <typename... Args>
void Log::debug(const char* message, Args&&... args) {
    getGlobalLogger().log<LogLevel::Debug>(message, std::forward<Args>(args)...);
}

template <typename... Args>
void Log::info(const char* message, Args&&... args) {
    getGlobalLogger().log<LogLevel::Info>(message, std::forward<Args>(args)...);
}   

template <typename... Args>
void Log::warning(const char* message, Args&&... args) {
    getGlobalLogger().log<LogLevel::Warning>(message, std::forward<Args>(args)...);
}

template <typename... Args>
void Log::error(const char* message, Args&&... args) {
    getGlobalLogger().log<LogLevel::Error>(message, std::forward<Args>(args)...);
}

template <typename... Args>
void Log::critical(const char* message, Args&&... args) {
    getGlobalLogger().log<LogLevel::Critical>(message, std::forward<Args>(args)...);
}

}

#endif