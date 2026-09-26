#include "control/robot.hpp"

namespace control
{

template <target::RobotTarget T>
Robot<T>::Robot(src::Drivers *drivers)
    : config_(drivers)
{
    config_.initialize();
}

template class Robot<target::RobotTarget::Standard>;
template class Robot<target::RobotTarget::Hero>;
template class Robot<target::RobotTarget::Sentry>;

}  // namespace control
