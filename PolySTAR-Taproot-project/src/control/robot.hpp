#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "config/control_config.hpp"
#include "robot_target.hpp"

namespace control {

template <target::RobotTarget T>
class Robot
{
public:
    explicit Robot(src::Drivers *drivers);

    void initialize();

private:
    ControlConfig<T> config_;
};

extern template class Robot<target::RobotTarget::Standard>;
extern template class Robot<target::RobotTarget::Hero>;
extern template class Robot<target::RobotTarget::Sentry>;

}


#endif