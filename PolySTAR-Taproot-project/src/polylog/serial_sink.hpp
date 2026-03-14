#ifndef SERIAL_SINK_HPP
#define SERIAL_SINK_HPP

#include "tap/communication/serial/uart.hpp"

#include "sink.hpp"

namespace polylog
{

template <tap::communication::serial::Uart::UartPort Port>
class SerialSink : public Sink
{
public:
    explicit SerialSink(Formatter* formatter, tap::communication::serial::Uart* uart);

    ~SerialSink() override;

    void setUart(tap::communication::serial::Uart* uart);

protected:
    void write(std::string_view text) override;

private:
    tap::communication::serial::Uart* uart_;
};

}  // namespace polylog

#include "serial_sink_impl.hpp"

#endif  // SERIAL_SINK_HPP