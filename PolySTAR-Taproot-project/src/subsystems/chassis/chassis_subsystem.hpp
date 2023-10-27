#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace control
{
namespace chassis
{
class ChassisSubsystem : public tap::control::Subsystem
{
public:

    ChassisSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers) {}

    ~ChassisSubsystem() = default;

    /**
     * Called once when the subsystem is added to the scheduler.
     */
    void initialize() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void refresh() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    static constexpr float CHASSIS_PID_KI = 0.2f;
    static constexpr float CHASSIS_PID_KD = 0.0f;
    static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 5000.0f;
    static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000.0f;
};

} // namespace chassis

} // namespace control

#endif  // CHASSIS_SUBSYSTEM_HPP_