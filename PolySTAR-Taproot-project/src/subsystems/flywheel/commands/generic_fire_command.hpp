#ifndef GENERIC_FIRE_COMMAND_HPP
#define GENERIC_FIRE_COMMAND_HPP

#include "tap/control/command.hpp"

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/core/flywheel_dji_subsystem.hpp"

namespace control::flywheel
{

template <typename Subsystem, typename FirePolicy>
class GenericFireCommand : public tap::control::Command
{
public:
    GenericFireCommand(Subsystem* const flywheel, src::Drivers* drivers)
        : tap::control::Command{},
          flywheel_{flywheel},
          drivers_{drivers},
          firePolicy_{flywheel, drivers}
    {
        if (flywheel == nullptr)
        {
            return;
        }
        this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(flywheel));
    }

    GenericFireCommand(const GenericFireCommand& other) = delete;

    ~GenericFireCommand() = default;

    GenericFireCommand& operator=(const GenericFireCommand& other) = delete;

    void initialize() override { firePolicy_.initialize(); }

    void execute() override { firePolicy_.execute(); }

    const char* getName() const override { return NAME; }

    bool isFinished() const override { return false; }

    void end(bool interrupt) override { firePolicy_.end(interrupt); }

private:
    static constexpr const char* NAME = "flywheel fire command";
    Subsystem* const flywheel_;
    src::Drivers* drivers_;
    FirePolicy firePolicy_;
};

}  // namespace control::flywheel

#endif