# inesctec_mrdt_serial_port

**Version 0.1.0**

This repository contains a library to handle serial communication. The library
is based on
[Boost.Asio](https://www.boost.org/doc/libs/1_80_0/doc/html/boost_asio.html).
As for the serial communication, the implementation is based on the example
`4_callback` provided in the
[serial-port](https://github.com/fedetft/serial-port) GitHub repository.

**With this version, it is possible to do:**

- Serial communication using Boost.Asio
- Set custom baud rate

**The next version will add these features:**

- Other possibilities of serial port communication (e.g., synchronous,
  low-level) based on the
  [serial-port](https://github.com/fedetft/serial-port) GitHub repository

## ROS

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

**ROS 2**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

### Dependencies

**ROS 1**

- [roscpp](https://wiki.ros.org/roscpp)
- [Boost](https://www.boost.org/)

**ROS 2**

- [Boost](https://www.boost.org/)

### Example

```cpp
#include <inesctec_mrdt_serial_port/AsyncSerial.h>

const unsigned int kSerialBaudRate = 115200;
const auto kSerialDataBits = boost::asio::serial_port_base::character_size(8);
const auto kSerialStopBits = boost::asio::serial_port_base::stop_bits::one;
const auto kSerialParity = boost::asio::serial_port_base::parity::none;
const auto kSerialFlowCtrl = boost::asio::serial_port_base::flow_control::none;

CallbackAsyncSerial* serial_async_;

bool openSerial() {
  try {
    serial_async_ = new CallbackAsyncSerial(
        "<SERIAL PORT NAME>", kSerialBaudRate,
        boost::asio::serial_port_base::parity(kSerialParity),
        kSerialDataBits,
        boost::asio::serial_port_base::flow_control(kSerialFlowCtrl),
        boost::asio::serial_port_base::stop_bits(kSerialStopBits));
    serial_async_->setCallback(
        std::bind(&Robot5dpoMSL::receiveSerialData, this,
                  std::placeholders::_1, std::placeholders::_2));
    return true;
  } catch (boost::system::system_error& e) {
    serial_async_ = nullptr;
    ROS_ERROR("Error when opening the serial device %s", "<SERIAL PORT NAME>");
    return false;
  }
}

void closeSerial() {
  if (serial_async_) {
    try {
      serial_async_->close();
    } catch(...) {
      ROS_ERROR("Error when closing the serial device %s",
                "<SERIAL PORT NAME>");
    }
    delete serial_async_;
  }
}
```

## Acknowledgements

- [@fedetft](https://github.com/fedetft)
- [5dpo Robotics Team](https://5dpo.github.io/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
