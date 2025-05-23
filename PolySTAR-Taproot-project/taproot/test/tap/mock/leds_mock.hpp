/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_LEDS_MOCK_HPP_
#define TAPROOT_LEDS_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/gpio/leds.hpp"

namespace tap
{
namespace mock
{
class LedsMock : public tap::gpio::Leds
{
public:
    LedsMock();
    virtual ~LedsMock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, set, (LedPin pin, bool isSet), (override));
};  // class LedsMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_LEDS_MOCK_HPP_
