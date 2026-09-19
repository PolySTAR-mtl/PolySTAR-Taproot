#ifndef IMU_INTERPRETER_HPP_
#define IMU_INTERPRETER_HPP_

#include <cstdint>

#include "control/drivers/drivers.hpp"

namespace algorithms
{

class ImuInterpreter
{
public:
    ImuInterpreter(src::Drivers *drivers);

    // Update the interpreter with new IMU data
    void update(const float xInput);

    // Get the current interpreted turret yaw RPM for stabilization
    float getTurretYawRPM() const;

private:
    src::Drivers *drivers;

    float turretYawRPM;

    std::uint32_t lastUpdateTime = 0;

    float chassisRotationSpeed = 0.f;

    std::uint32_t gzSamplingCount = 0;
    float gzSamplingSum = 0.f;
};

} // namespace algorithms

#endif // IMU_INTERPRETER_HPP_