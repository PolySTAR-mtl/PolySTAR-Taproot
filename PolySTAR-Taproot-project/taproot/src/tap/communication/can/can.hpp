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

#ifndef TAPROOT_CAN_HPP_
#define TAPROOT_CAN_HPP_

#include "tap/util_macros.hpp"

#include "can_bus.hpp"

namespace modm::can
{
class Message;
}

namespace tap
{
namespace can
{
/**
 * A simple CAN wrapper class that handles I/O from both CAN bus 1 and 2.
 */
class Can
{
public:
    Can() = default;
    DISALLOW_COPY_AND_ASSIGN(Can)
    mockable ~Can() = default;

    /**
     * Initializes CAN 1 and CAN 2 hardware to pins specific to the development
     * board and sets up the CAN bus filters necessary for reading
     * from the CAN bus.
     *
     * @note CAN 1 is connected to pins D0 (RX) and D1 (TX) and
     *      CAN 2 is connected to pins B12 (RX) and B12 (TX).
     * @note The CAN filters are set up to receive NOT extended identifier IDs.
     */
    mockable void initialize();

    /**
     * Checks the passed in CanBus to see if there is a message waiting
     * and available.
     *
     * @param[in] bus the CanBus to check for a message.
     * @return true if a message is available, false otherwise.
     */
    mockable bool isMessageAvailable(CanBus bus) const;

    /**
     * Checks the CanBus for a message and if a message is successfully
     * acquired, returns true and places the message in the return parameter
     * message.
     *
     * @param[in] bus the CanBus to acquire a message from.
     * @param[out] message a return parameter which the message is
     *      placed in.
     * @return true if a valid message was placed in the parameter
     *      message. False otherwise.
     */
    mockable bool getMessage(CanBus bus, modm::can::Message *message);

    /**
     * Checks the given CanBus to see if the CanBus is idle.
     *
     * @param[in] bus the CanBus to check.
     * @return true if the bus is not busy, false otherwise.
     */
    mockable bool isReadyToSend(CanBus bus) const;

    /**
     * Sends the passed in message over the CanBus. Returns whether or
     * not the message succeeded.
     *
     * @attention `modm::can::Message` defaults to an extended
     * message identifier. For all RoboMaster products we have, we do not
     * want our messages to be extended. Be sure to be explicit
     * when instantiating a message object and setting extended to
     * false.
     *
     * @param[in] bus the `CanBus` for which the message should be sent across.
     * @param[in] message the message to send
     * @return true if the message was successfully sent, false otherwise.
     */
    mockable bool sendMessage(CanBus bus, const modm::can::Message &message);
};  // class Can

}  // namespace can
}  // namespace tap

#endif  // TAPROOT_CAN_HPP_