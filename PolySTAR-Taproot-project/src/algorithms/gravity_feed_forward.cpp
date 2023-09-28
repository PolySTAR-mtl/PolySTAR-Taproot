#include "gravity_feed_forward.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace src
{
namespace algorithms
{

GravityFeedForward::GravityFeedForward(const FeedForwardConfig &ffConfig, float x, float y): 
    FeedForward(ffConfig), 
    offsetAngle(atanf(y/x)) 
{}

float GravityFeedForward::calculate(float velocity, float angleDeg) {
    velocity = tap::algorithms::limitVal<float>(velocity, -config.maxVelocity, config.maxVelocity);
    return config.ks * tap::algorithms::getSign(velocity) + config.kv * velocity + config.kg * cosf((angleDeg*DEG_TO_RAD) + offsetAngle);
}

}  // namespace algorithms

}  // namespace src
