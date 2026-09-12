#include "feed_forward.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace src
{
namespace algorithms
{
FeedForward::FeedForward(const FeedForwardConfig &ffConfig)
    : config_(ffConfig)
{
}

float FeedForward::calculate(float velocity)
{
    velocity = tap::algorithms::limitVal<float>(velocity, -config_.maxVelocity, config_.maxVelocity);
    return config_.ks * tap::algorithms::getSign(velocity) + config_.kv * velocity;
}


void FeedForward::setKs(float ks)
{
    config_.ks = ks;
}

void FeedForward::setKv(float kv)
{
    config_.kv = kv;
}

void FeedForward::setKg(float kg) 
{
    config_.kg = kg;
}

void FeedForward::setMaxVel(float maxVel)
{
    config_.maxVelocity = maxVel;
}

}  // namespace algorithms

}  // namespace src
