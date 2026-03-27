#ifndef CHASSIS_TEST_DJI_MOTORS_HPP_
#define CHASSIS_TEST_DJI_MOTORS_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/architecture/timeout.hpp"

namespace control
{
namespace chassis
{
class ChassisTestDjiMotorsCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisTestDjiMotorsCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisTestDjiMotorsCommand(const ChassisTestDjiMotorsCommand &other) = delete;

    ChassisTestDjiMotorsCommand &operator=(const ChassisTestDjiMotorsCommand &other) = delete;

    const char *getName() const { return "chassis test DJI motors command"; }

    void initialize() override;

    virtual void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    tap::arch::MilliTimeout startMatchTimeout;

protected:
    ChassisSubsystem *const chassis;
    
    src::Drivers *drivers;
};  // ChassisTestDjiMotorsCommand

}  // namespace chassis

}  // namespace control

#endif // CHASSIS_TEST_DJI_MOTORS_HPP_