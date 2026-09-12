#ifndef TURRET_CONVERSIONS_HPP
#define TURRET_CONVERSIONS_HPP

#include <cstdint>

namespace control::turret {

inline constexpr uint16_t degreesToTicks(float degrees);

} // namespace control::turret

#include "turret_conversions_impl.hpp"

#endif // TURRET_CONVERSIONS_HPP