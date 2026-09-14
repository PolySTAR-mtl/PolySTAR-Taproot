#ifndef IMU_INTERPRETER_HPP_
#define IMU_INTERPRETER_HPP_

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

    uint32_t compoundedTime = 0;
    float chassisRotationSpeed = 0;
    int gzSamplingCount = 0;
    float gzSamplingSum = 0;
    float gzAverage = 0;
};

} // namespace algorithms

#endif // IMU_INTERPRETER_HPP_