#ifndef TURRET_YAW_CONTROLLER_HPP_
#define TURRET_YAW_CONTROLLER_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "algorithms/feed_forward.hpp"

namespace turret
{
namespace algorithms
{

class TurretYawController
{
public:
    TurretYawController(const tap::algorithms::SmoothPidConfig &pidConfig, const src::algorithms::FeedForwardConfig &ffConfig);

    void runController(float error, float errorDerivative, float velocity, float dt);

    void runControllerDerivateError(float error, float velocity, float dt);

    float getOutput();

    // Feed forward setters
    inline void setKs(float ks) { yawFeedForward.setKs(ks); }
    inline void setKv(float kv) { yawFeedForward.setKv(kv); }
    inline void setKg(float kg) { yawFeedForward.setKg(kg); }
    inline void setMaxVel(float maxVel) { yawFeedForward.setMaxVel(maxVel); }

    // PID setters
    inline void setP(float p) { yawPid.setP(p); }
    inline void setI(float i) { yawPid.setI(i); }
    inline void setD(float d) { yawPid.setD(d); }
    inline void setMaxICumulative(float maxICumulative) { yawPid.setMaxICumulative(maxICumulative); }
    inline void setMaxOutput(float maxOutput) { yawPid.setMaxOutput(maxOutput); }
    inline void setErrDeadzone(float errDeadzone) { yawPid.setErrDeadzone(errDeadzone); }

    inline void reset() { yawPid.reset(); }

private:
    tap::algorithms::SmoothPid yawPid;
    src::algorithms::FeedForward yawFeedForward;

    float output;
    float maxOutput;
};

}  // namespace algorithms

}  // namespace src

#endif  // TURRET_YAW_CONTROLLER_HPP_
