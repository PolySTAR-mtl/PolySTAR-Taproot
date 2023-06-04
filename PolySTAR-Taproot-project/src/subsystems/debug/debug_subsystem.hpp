#ifndef DEBUG_SUBSYSTEM_HPP_
#define DEBUG_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "debug_constants.hpp"

namespace control
{
namespace debug
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */
class DebugSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new TurretSubsystem with default parameters specified in
     * the private section of this class.
     */
    DebugSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers)
    {
    }

    DebugSubsystem(const DebugSubsystem &other) = delete;

    DebugSubsystem &operator=(const DebugSubsystem &other) = delete;

    ~DebugSubsystem() = default;

    void initialize() override;

    void refresh() override;

private:

    uint32_t prevDebugTime;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
