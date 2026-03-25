#include "log.hpp"

namespace polylog
{

DefaultLogger& defaultLogger() {
    static DefaultLogger logger("Default");
    return logger;
}

}