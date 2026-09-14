#ifndef GENERIC_AIM_COMMAND_HPP
#define GENERIC_AIM_COMMAND_HPP

#include "tap/control/command.hpp"
#include "control/drivers/drivers.hpp"

#include "subsystems/concepts/command_policy.hpp"


namespace control::turret
{

template <typename Subsystem, command_policy AimPolicy>
class GenericAimCommand : public tap::control::Command
{
public:

    GenericAimCommand(Subsystem* const turret, src::Drivers* drivers);

    ~GenericAimCommand();

    GenericAimCommand(const GenericAimCommand& other) = delete;

    GenericAimCommand& operator=(const GenericAimCommand& other) = delete;

    void initialize() override;

    void execute() override;

    const char* getName() const override;

    bool isFinished() const override;

    void end(const bool interrupt) override;

private:
    static constexpr const char* NAME = "turret aim command";
    Subsystem* const turret_;
    src::Drivers* drivers_;
    AimPolicy aimPolicy_;
};

} // namespace control::turret

#include "generic_aim_command_impl.hpp"

#endif // GENERIC_AIM_COMMAND_HPP