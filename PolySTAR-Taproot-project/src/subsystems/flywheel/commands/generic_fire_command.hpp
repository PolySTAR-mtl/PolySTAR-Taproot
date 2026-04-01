#ifndef GENERIC_FIRE_COMMAND_HPP
#define GENERIC_FIRE_COMMAND_HPP

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/core/flywheel_dji_subsystem.hpp"
#include "subsystems/flywheel/concepts/fire_policy.hpp"

namespace control::flywheel
{

template <typename Subsystem, fire_policy FirePolicy>
class GenericFireCommand : public tap::control::Command
{
public:
    GenericFireCommand(Subsystem* const flywheel, src::Drivers* drivers);

    ~GenericFireCommand();
    
    GenericFireCommand(const GenericFireCommand& other) = delete;

    GenericFireCommand& operator=(const GenericFireCommand& other) = delete;

    void initialize() override;

    void execute() override;

    const char* getName() const override;

    bool isFinished() const override;

    void end(const bool interrupt) override;

private:
    static constexpr const char* NAME = "flywheel fire command";
    Subsystem* const flywheel_;
    src::Drivers* drivers_;
    FirePolicy firePolicy_;
};

}  // namespace control::flywheel

#include "generic_fire_command_impl.hpp"

#endif