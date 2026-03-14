#ifndef SERIAL_SINK_IMPL_HPP
#define SERIAL_SINK_IMPL_HPP

#include "serial_sink.hpp"

namespace polylog
{

template <tap::communication::serial::Uart::UartPort Port>
SerialSink<Port>::SerialSink(Formatter* formatter, tap::communication::serial::Uart* uart)
    : Sink{formatter},
      uart_{uart}
{
}

template <tap::communication::serial::Uart::UartPort Port>
SerialSink<Port>::~SerialSink() = default;

template <tap::communication::serial::Uart::UartPort Port>
void SerialSink<Port>::setUart(tap::communication::serial::Uart* uart)
{
    uart_ = uart;
}

template <tap::communication::serial::Uart::UartPort Port>
void SerialSink<Port>::write(std::string_view text)
{
    if (uart_ == nullptr || text.empty()) return;
    uart_->write(Port, reinterpret_cast<const uint8_t*>(text.data()), text.size());
}

}  // namespace polylog

#endif  // SERIAL_SINK_IMPL_HPP