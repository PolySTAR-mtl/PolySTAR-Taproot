#ifndef GENERIC_FIRE_COMMAND_IMPL_HPP
#define GENERIC_FIRE_COMMAND_IMPL_HPP

#include "generic_fire_command.hpp"

namespace control::flywheel
{

template <typename Subsystem, command_policy FirePolicy>
GenericFireCommand<Subsystem, FirePolicy>::GenericFireCommand(Subsystem* const flywheel, src::Drivers* drivers)
    : tap::control::Command{},
        flywheel_{flywheel},
        drivers_{drivers},
        firePolicy_{flywheel}
{
    if (flywheel == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(flywheel));
}

template <typename Subsystem, command_policy FirePolicy>
GenericFireCommand<Subsystem, FirePolicy>::~GenericFireCommand() = default;

template <typename Subsystem, command_policy FirePolicy>
void GenericFireCommand<Subsystem, FirePolicy>::initialize()
{
    firePolicy_.initialize();
}

template <typename Subsystem, command_policy FirePolicy>
void GenericFireCommand<Subsystem, FirePolicy>::execute()
{
    firePolicy_.execute();
}

template <typename Subsystem, command_policy FirePolicy>
const char* GenericFireCommand<Subsystem, FirePolicy>::getName() const
{
    return NAME;
}

template <typename Subsystem, command_policy FirePolicy>
bool GenericFireCommand<Subsystem, FirePolicy>::isFinished() const
{
    return false;
}

template <typename Subsystem, command_policy FirePolicy>
void GenericFireCommand<Subsystem, FirePolicy>::end(const bool interrupt)
{
    firePolicy_.end(interrupt);
}

} // namespace control::flywheel

#endif // GENERIC_FIRE_COMMAND_IMPL_HPP