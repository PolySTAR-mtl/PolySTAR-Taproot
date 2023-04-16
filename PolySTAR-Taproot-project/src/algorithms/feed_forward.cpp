#include "feed_forward.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace src
{
namespace algorithms
{
FeedForward::FeedForward(const FeedForwardConfig &ffConfig)
    : config(ffConfig)
{
}

float FeedForward::calculate(float velocity) {
    velocity = tap::algorithms::limitVal<float>(velocity, -config.maxVelocity, config.maxVelocity);
    return config.ks * tap::algorithms::getSign(velocity) + config.kv * velocity;
}

float FeedForward::calculateWithGravity(float velocity, float angle) {
    velocity = tap::algorithms::limitVal<float>(velocity, -config.maxVelocity, config.maxVelocity);
    return config.ks * tap::algorithms::getSign(velocity) + config.kv * velocity + config.kg * cosf(angle*DEG_TO_RAD);
}

}  // namespace algorithms

}  // namespace src
