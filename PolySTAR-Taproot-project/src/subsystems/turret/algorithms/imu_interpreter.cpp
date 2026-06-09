#include "subsystems/turret/algorithms/imu_interpreter.hpp"
#include "subsystems/turret/config/turret_constants.hpp"

namespace algorithms
{
    ImuInterpreter::ImuInterpreter(src::Drivers *drivers)
        : drivers(drivers)
        , turretYawRPM(0)
        , compoundedTime(0)
        , chassisRotationSpeed(0)
        , gzSamplingCount(0)
        , gzSamplingSum(0)
        , gzAverage(0) {}

    void ImuInterpreter::update(const float xInput) {
        float gZ = drivers->mpu6500.getGz();
        gzSamplingSum += gZ;
        gzSamplingCount++;
        gzAverage = gzSamplingSum / gzSamplingCount;

        uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
        uint32_t timeDelta = currentUpdate - compoundedTime;
        compoundedTime = currentUpdate;

        compoundedTime += timeDelta;
        if (compoundedTime >= 20) {
            compoundedTime = 0;
            if (abs(gzAverage) > 0.5f) {
                chassisRotationSpeed = gzAverage;
            }
            else {
                chassisRotationSpeed = 0;
            }

            gzAverage = drivers->mpu6500.getGz();
            gzSamplingSum = 0;
            gzSamplingCount = 0;
        }

        turretYawRPM = ((GZ_STABILIZATION_CONSTANT - X_INPUT_STABILIZATION_CONSTANT * xInput) * chassisRotationSpeed);
    }

    float ImuInterpreter::getTurretYawRPM() const {
        return turretYawRPM;
    }
} // namespace algorithms