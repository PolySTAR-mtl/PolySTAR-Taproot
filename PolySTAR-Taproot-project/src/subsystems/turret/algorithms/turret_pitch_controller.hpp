#ifndef TURRET_PITCH_CONTROLLER_HPP_
#define TURRET_PITCH_CONTROLLER_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "algorithms/feed_forward.hpp"

namespace turret
{
namespace algorithms
{

class TurretPitchController
{
public:
    TurretPitchController(const tap::algorithms::SmoothPidConfig &pidConfig, const src::algorithms::FeedForwardConfig &ffConfig);

    void runController(float error, float errorDerivative, float velocity, float angle, float dt);

    void runControllerDerivateError(float error, float velocity, float angle, float dt);

    float getOutput();

    // Feed forward setters
    inline void setKs(float ks) { pitchFeedForward.setKs(ks); }
    inline void setKv(float kv) { pitchFeedForward.setKv(kv); }
    inline void setKg(float kg) { pitchFeedForward.setKg(kg); }
    inline void setMaxVel(float maxVel) { pitchFeedForward.setMaxVel(maxVel); }

    // PID setters
    inline void setP(float p) { pitchPid.setP(p); }
    inline void setI(float i) { pitchPid.setI(i); }
    inline void setD(float d) { pitchPid.setD(d); }
    inline void setMaxICumulative(float maxICumulative) { pitchPid.setMaxICumulative(maxICumulative); }
    inline void setMaxOutput(float maxOutput) { pitchPid.setMaxOutput(maxOutput); }
    inline void setErrDeadzone(float errDeadzone) { pitchPid.setErrDeadzone(errDeadzone); }

    inline void reset() { pitchPid.reset(); }

private:
    tap::algorithms::SmoothPid pitchPid;
    src::algorithms::FeedForward pitchFeedForward;

    float output;
    float maxOutput;
};

}  // namespace algorithms

}  // namespace src

#endif  // TURRET_PITCH_CONTROLLER_HPP_
