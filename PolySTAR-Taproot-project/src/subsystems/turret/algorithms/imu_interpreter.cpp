#include "subsystems/turret/algorithms/imu_interpreter.hpp"

#include <numbers>

#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace algorithms
{
    ImuInterpreter::ImuInterpreter(src::Drivers *drivers)
        : drivers{drivers}
        , turretYawRPM{0.f}
        , compoundedTime{0}
        , chassisRotationSpeed{0.f}
        , gzSamplingCount{0}
        , gzSamplingSum{0.f}
        , gzAverage{0.f}
    {}

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
            static constexpr float GZ_THRESHOLD_RADIANS_PER_SECOND = 0.5f * std::numbers::pi_v<float> / 180.0f; // Threshold for considering the Gz value significant
            if (abs(gzAverage) > GZ_THRESHOLD_RADIANS_PER_SECOND) {
                chassisRotationSpeed = gzAverage;
            }
            else {
                chassisRotationSpeed = 0;
            }

            gzAverage = drivers->mpu6500.getGz();
            gzSamplingSum = 0;
            gzSamplingCount = 0;
        }

        turretYawRPM = (control::turret::ACTIVE_TURRET_CONFIG.gzStabilizationFactor - control::turret::X_INPUT_STABILIZATION_CONSTANT * xInput) * chassisRotationSpeed;
    }

    float ImuInterpreter::getTurretYawRPM() const {
        return turretYawRPM;
    }
} // namespace algorithms