#ifndef SUBSYSTEM_HPP
#define SUBSYSTEM_HPP

#include <concepts>

#include "tap/control/subsystem.hpp"

namespace polystar
{
    template <typename T>
    concept subsystem_derived = std::derived_from<T, tap::control::Subsystem>;
}

#endif // SUBSYSTEM_HPP