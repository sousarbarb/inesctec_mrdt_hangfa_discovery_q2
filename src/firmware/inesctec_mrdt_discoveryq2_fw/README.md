# inesctec_mrdt_discoveryq2_fw

This repository implements the firmware relative to the Hangfa Discovery Q2
four-wheeled omnidirectional robotic platform.

**Version 2.2.0**

**With this version, it is possible to do:**

- Serial communication using the library channels
- Read encoders channel A and B for wheeled odometry
- Motors angular speed control with a generic PID controller
- Motors PWM control
  ([TimerOne](https://github.com/PaulStoffregen/TimerOne) and
  [TimerThree](https://github.com/PaulStoffregen/TimerThree) PWM)
- Watchdog timer to disable the motors if the PC does not send anything for a
  certain time
- Limitation on the PWM change

**The next version will add these features:**

- Tune the PID controllers for angular speed control of the motors

**Bugs identified in the current version:**

- None

## Hardware

- Hangfa Discovery Q2 Platform
- Faulhaber DC Motor 2342 OEM
  ([datasheet [not possible to validate this link...]](https://www.faulhaber.com/fileadmin/Import/Media/EN_2342_CR_DFF.pdf))
  - 3K3 Ohms Resistors (pull-up resistor for the encoders channels)
- Arduino Mega 2560 Rev3
- Arduino Mega Proto Shield Rev3
  - IDC Socket 6Way
    ([store](https://mauser.pt/catalog/product_info.php?products_id=011-0827))
  - IDC Header Straight 6Way
    ([store](https://mauser.pt/catalog/product_info.php?products_id=011-0800))
  - 8-pin stackable header (Arduino Compatible)
    ([store](https://opencircuit.nl/product/female-header-stackable-8-pin-10-stuks))
  - 10-pin stackable header (Arduino Compatible)
    ([store](https://opencircuit.nl/product/female-header-stackable-10-pin-10-stuks))
  - 2x18-pin stackable header (Arduino Compatible)
    ([store](https://opencircuit.nl/product/female-header-stackable-2x18-pin-2-stuks))
- Cytron 13A, 5-30V Single DC Motor Controller
  - KK shell connector 3Way 2.54mm (Molex 5051 Housing)
    ([store](https://mauser.pt/catalog/product_info.php?products_id=011-1319))
  - Terminals KK shell connector 3Way 2.54mm
    ([store](https://mauser.pt/catalog/product_info.php?products_id=011-3065))
- Li-Ion 18650 BMS Charger PCB, 3S 12.6V 20A
- Tattu 11.1V 15C 3S 10000mAh Lipo Battery Pack

## Serial Communication (channels)

**Robot > PC**

- `[g]`: encoders total count of motor 0 (ticks)
- `[h]`: encoders total count of motor 1 (ticks)
- `[i]`: encoders total count of motor 2 (ticks)
- `[j]`: encoders total count of motor 3 (ticks)
- `[k]`: interval time between control cycles (us)
- `[r]`: reset signal

**PC > Robot**

- `[G]`: reference angular speed of motor 0 (rad/s)
- `[H]`: reference angular speed of motor 1 (rad/s)
- `[I]`: reference angular speed of motor 2 (rad/s)
- `[J]`: reference angular speed of motor 3 (rad/s)
- `[K]`: PWM value of motors
  - `(value >> 24) & 0x03`: motor index (0..3)
  - `value & 0xFFFF`: PWM value (0..`kMotPWMmax`)

## Usage

**Requirements**

- PlatformIO extension for VS Code

### Compilation

**Visual Studio Code**

1. Clone the repository
    ```sh
    git clone git@github.com:sousarbarb/inesctec_mrdt_hangfa_discovery_q2.git
    cd inesctec_mrdt_hangfa_discovery_q2/src/firmware/inesctec_mrdt_discoveryq2_fw
    code .
    ```
2. Compile the project using the PlatformIO extension in VS Code
   (PlatformIO on left-side navigation bar > Project Tasks > Build)

### Run

1. Open the PlatformIO project in the Visual Studio Code
2. Upload the project using the PlatformIO extension in VS Code
   (PlatformIO on left-side navigation bar > Project Tasks > Upload)

## Acknowledgements

- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
