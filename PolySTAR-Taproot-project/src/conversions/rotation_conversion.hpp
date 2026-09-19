#ifndef ROTATION_CONVERSION_HPP
#define ROTATION_CONVERSION_HPP

#include <numbers>

namespace conversions {

template <typename T>
inline constexpr T degreesToRadians(T degrees) {
    return degrees * (std::numbers::pi_v<T> / 180.f);
}

template <typename T>
inline constexpr T radiansToDegrees(T radians) {
    return radians * (180.f / std::numbers::pi_v<T>);
}

template <typename T>
inline constexpr T rpmToRadiansPerSecond(T rpm) {
    return rpm * 2.f * std::numbers::pi_v<T> / 60.f;
}

template <typename T>
inline constexpr T radiansPerSecondToRpm(T radiansPerSecond) {
    return radiansPerSecond * 30.f / std::numbers::pi_v<T>;
}

} // namespace conversions

#endif // ROTATION_CONVERSION_HPP