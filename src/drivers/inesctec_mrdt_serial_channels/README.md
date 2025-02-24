# inesctec_mrdt_serial_channels

**Version 1.0**

This folder contains a ROS package that defines a library to communicate with
another devices such as Arduinos (Uno, Mega, etc). The communication protocol is
based on the _channels_ library developed by professor Paulo Costa. The current
version is an improvement over the one developed by Héber Miguel Sobreira
(https://gitlab.inesctec.pt/heber.m.sobreira/serial_communication_channels.git)
in the way that allows channels between 'g'-'z' and 'G'-'Z'. An important note
is that you must check if on the Arduino side the hexdecimals characters
correspond to the ones consider in this package: '0'..'9','A'..'F'.

Lastly, it is possible to use the same library on the Arduino (example provided
by Héber Miguel Sobreira - please contact him through email
[heber.m.sobreira@inesctec.pt](mailto:heber.m.sobreira@inesctec.pt)
for further informations on this matter).
However, this implementation uses reallocs of memory. Even though it is only
when the Arduino turns on, I did not think that this approach would be the
correct one due to Arduinos do not have a proper memory management system. So,
the one used on the Arduino is the same one developed by professor Paulo Costa.
**Main disadvantage:** the Arduino only interprets fixed-size packets
(specifically, 4 bytes of data: `float`, `uint32_t` or `int32_t`).

**With this version, it is possible to:**

- Send data of the following types: `float`, `uint32_t` or `int32_t`
  (limited because of the implementation on the Arduino side)
- 40 different channels with different functions associated with each one
  ('g'-'z' and 'G'-'Z')

**The next version will add these features:**

- Binary write and read (compression that allows to reduce from 8 to 4 bytes of
  data on certain data types)

## ROS

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

**ROS 2**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

## Communication Protocol

- `[CHANNEL]` + `[DATA]` + `\0`
- `[CHANNEL]`: 'g'-'z', 'G'-'Z'
- `[DATA]`: characters that represent a value in hexadecimal

## Acknowledgements

- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [Héber Miguel Sobreira](https://gitlab.inesctec.pt/heber.m.sobreira/)
- [Paulo G. Costa](https://github.com/P33a)
