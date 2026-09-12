#ifndef TURRET_CONVERSIONS_IMPL_HPP
#define TURRET_CONVERSIONS_IMPL_HPP

#include <cstdint>

#include "tap/motor/dji_motor_encoder.hpp"

namespace control::turret {

static constexpr float DEGREE_TO_TICK =
    tap::motor::DjiMotorEncoder::ENC_RESOLUTION / 360.0f; // 8192 Ticks per turn, 1:1 gear ratio

constexpr uint16_t degreesToTicks(float degrees) {
    return static_cast<uint16_t>(degrees * DEGREE_TO_TICK);
}

} // namespace control::turret

#endif // TURRET_CONVERSIONS_IMPL_HPP