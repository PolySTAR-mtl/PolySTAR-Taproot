#ifndef LOG_HPP
#define LOG_HPP

#include "tap/communication/serial/uart.hpp"

#include "log_level.hpp"

namespace polylog
{

#ifdef LOG_LEVEL
constexpr LogLevel GLOBAL_LOG_LEVEL = LOG_LEVEL;
#else
constexpr LogLevel GLOBAL_LOG_LEVEL = LogLevel::Info;
#endif

class Log {
public:
using GlobalLogger = Logger<GLOBAL_LOG_LEVEL>;
public:
    static void init(tap::communication::serial::Uart* uart);
    
    template <typename... Args>
    static void trace(const char* message, Args&&... args);

    template <typename... Args>
    static void debug(const char* message, Args&&... args);

    template <typename... Args>
    static void info(const char* message, Args&&... args);

    template <typename... Args>
    static void warning(const char* message, Args&&... args);

    template <typename... Args>
    static void error(const char* message, Args&&... args);

    template <typename... Args>
    static void critical(const char* message, Args&&... args);

private:
    static GlobalLogger& getGlobalLogger();
};







}  // namespace polylog

#include "log_impl.hpp"

#endif  // LOG_HPP