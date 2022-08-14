#include "safe_disconnect.hpp"

#include "tap/control/command_scheduler.hpp"

#include "control/drivers/drivers.hpp"

namespace src::control
{
RemoteSafeDisconnectFunction::RemoteSafeDisconnectFunction(src::Drivers *drivers)
{
    this->drivers = drivers;
}
bool RemoteSafeDisconnectFunction::operator()() { return !drivers->remote.isConnected(); }
}  // namespace aruwsrc::control
