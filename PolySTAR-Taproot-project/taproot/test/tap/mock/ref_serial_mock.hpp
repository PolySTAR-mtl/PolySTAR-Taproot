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

#ifndef TAPROOT_REF_SERIAL_MOCK_HPP_
#define TAPROOT_REF_SERIAL_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

namespace tap
{
namespace mock
{
class RefSerialMock : public tap::communication::serial::RefSerial
{
public:
    RefSerialMock(Drivers* drivers);
    virtual ~RefSerialMock();

    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const tap::communication::serial::DJISerial::ReceivedSerialMessage&),
        (override));
    MOCK_METHOD(bool, getRefSerialReceivingData, (), (const override));
    MOCK_METHOD(const Rx::RobotData&, getRobotData, (), (const override));
    MOCK_METHOD(const Rx::GameData&, getGameData, (), (const override));
    MOCK_METHOD(
        void,
        attachRobotToRobotMessageHandler,
        (uint16_t, RobotToRobotMessageHandler*),
        (override));
    MOCK_METHOD(RobotId, getRobotIdBasedOnCurrentRobotTeam, (RobotId), (override));
    MOCK_METHOD(void, acquireTransmissionSemaphore, (), (override));
    MOCK_METHOD(void, releaseTransmissionSemaphore, (), (override));
};  // class RefSerialMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_REF_SERIAL_MOCK_HPP_