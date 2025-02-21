# LattePanda Sigma

## Links

- [LattePanda Sigma](https://www.lattepanda.com/lattepanda-sigma)
- [LattePanda Specification](https://docs.lattepanda.com/content/sigma_edition/Specification/)
- [Pinout and Hardware Diagram](https://docs.lattepanda.com/content/sigma_edition/IO_Playability/)
- [Initial Setup of the Sigma by LattePanda](https://docs.lattepanda.com/content/sigma_edition/Getting_Started/)
- [LattePanda Sigma 3D Model](https://github.com/LattePandaTeam/LattePanda-3D-Models/tree/master/LattePanda%20Sigma)

## Specifications

| Component        | Content                    | Specs.                                                       |
| :--------------- | -------------------------- | :----------------------------------------------------------- |
| Processor        | CPU                        | Intel® Core™ i5-1340P                                        |
|                  | Cores / Threads            | 12C(4P+8E) / 16T                                             |
|                  | Max Turbo Frequency        | 4.60 GHz(Performance-core), 3.4 GHz(Efficient-core)          |
|                  | L2 Cache                   | 12 MB                                                        |
|                  | Base Power                 | 28 W  <br>(BIOS Default: PL1 45W, PL2 64W)                   |
|                  | Graphics                   | Intel® Iris® Xe Graphics                                     |
|                  | Max Dynamic Frequency      | 1.45 GHz                                                     |
|                  | Execution Units            | 80                                                           |
|                  | Co-processor               | Microchip® ATmega32U4-MU                                     |
| Memory           | Technology                 | Dual-Channel, Samsung LPDDR5-6400(16GB), LPDDR5-6000(32GB)   |
|                  | Capacity                   | 16GB，32GB                                                   |
| Storage          | Drive Form Factor          | M.2 SSD (NVMe/SATA), SATA Drive                              |
| Wireless         | Wireless Form Factor       | M.2 Wireless Module (PCIe/CNVio)                             |
| Display          | HDMI Port                  | HDMI 2.1, Up to 4096 x 2304 @ 60Hz                           |
|                  | USB Type-C Port            | DP 1.4a, Up to 7680 x 4320 @ 60Hz (One Monitor)              |
|                  | eDP(Embedded Display Port) | eDP 1.4b, 4 Lanes, Up to 4096 x 2304 @ 120Hz                 |
| External I/O     | USB Type-A                 | 2 x USB2.0 (480Mbps), 2 x USB3.2 Gen 2 (10Gbps)              |
|                  | USB Type-C                 | 2 x Thunderbolt™ 4 (40Gbps)                                  |
|                  | HDMI                       | HDMI 2.1                                                     |
|                  | Ethernet                   | 2 x 2.5GbE RJ45 Ports (Intel® i225-V Gigabit Ethernet Controller, Supports 10/100/1000/2500 Mbps, WOL) |
|                  | Power                      | 5.5mm x 2.5mm DC Jack                                        |
|                  | Audio                      | 3.5mm Microphone Headphone Combo Connector                   |
|                  | Sim Card                   | Micro Sim Card Slot                                          |
| Internal I/O     | USB 2.0                    | 2.0mm Pitch 4-Pin Connector, 480Mbps                         |
|                  | Fan                        | 1.27mm Pitch,12V 4-Wire Fan Connector, PWM Control           |
|                  | SATA                       | SATA 6.0 Gb/s Data Connector, 2.0mm Pitch 4-Pin Power Connector |
|                  | Front Panel                | 2.54mm Pitch 9-Pin Header, Supports Power, Reset, Power LED, HDD LED |
|                  | Front Audio Panel          | 2.54mm Pitch 9-Pin Header, Supports High Definition Audio (HD), Line-Out, Mic-in |
|                  | COM                        | 2.54mm Pitch 9-Pin Header,Supports RS232, RS485              |
|                  | GPIO                       | 2.54mm Pitch 34-Pin Header, Including ATmega32U4 I/O Pins, 5V Power Pins, S0/S3/S4 State Pins |
|                  | eDP                        | 0.5mm Pitch 40-Pin Connector, 4 Lanes                        |
| Expansion Slot   | M.2 M Key                  | Type 2280, Supports PCIe 3.0 x4<br>Type 2280, Supports PCIe 4.0 x4 |
|                  | M.2 B Key                  | Type 2242/2252/2280, Supports SATA III/PCIe 3.0 x1, USB2.0, USB3.0, SIM |
|                  | M.2 E Key                  | Type 2230, Supports PCIe 3.0 x1,USB2.0, Intel CNVio          |
| Security         | TPM                        | Processor's Built-in TPM (2.0)                               |
| Power            | Power Input                | 5.5mm x 2.1mm DC Jack: DC 12~20V <br>USB Type C: PD 20V      |
|                  | Attached Power Adapter     | 19V DC, 4.74A, 90W                                           |
|                  | RTC Battery                | CR1220 Battery Holder: 3V <br>1.27mm Pitch 2-Pin Connector: 3V |
| Operating System | Microsoft Windows          | Windows 10, Windows 11                                       |
|                  | Linux                      | Ubuntu, Promox VE, etc.                                      |
| Environment      | Operating Temperature      | 0~45℃                                                        |
|                  | Relative Humidity          | 0%~80%RH                                                     |
| Dimension        | Form Factor                | 3.5", 146 x 102 mm                                           |
| Model(SKU)       | DFR1080                    | LattePanda Sigma (16GB RAM, no SSD, no WiFi)                 |
|                  | DFR1081                    | LattePanda Sigma (16GB RAM, 500GB PCIe 4.0 x4 NVMe SSD, Intel AX211 WiFi6E) |
|                  | DFR1090                    | LattePanda Sigma (32GB RAM, no SSD, no WiFi)                 |
|                  | DFR1091                    | LattePanda Sigma (32GB RAM, 500GB PCIe 4.0 x4 NVMe SSD, Intel AX211 WiFi6E) |
| Compliance       | Certification              | CE, FCC, KC, RoHS                                            |

## Initial Setup

- :fontawesome-regular-hard-drive: M.2 NVMe SSD
- :material-cable-data:            HDMI Cable
- :material-monitor:               External HDMI Monitor
- :material-keyboard:              Keyboard and a Mouse
- :material-power-plug-battery:    Power Adapter
- :material-antenna:               M.2 Wireless Network Model with WIFI/BT antennas
    - M.2 (A+E Key) AX210 WiFi 6E Network Card for LattePanda Sigma, Alpha, Delta

**Instructions**

1. Connect an M.2 NVMe SSD to the M.2 Key-M socket
2. Install a compatible M.2 wireless network card
    - Intel AX210
    - Intel AX211 _(includes in the LattePanda Sigma 32GB bundle DFR1091)_
    - Intel AX200
    - Intel AX201
    - Intel AC9560
    - Intel AC8265
3. _(optional)_ Install the LattePanda Sigma into a case
    - Aluminum Case for LattePanda Sigma Single Board Computer
    - Vertical M.2 Expansion Support Case in Aluminum Alloy
4. Connect an HDMI display
5. Connect the keyboard and mouse to the LattePanda 3 Delta
6. Connect the power adapter provided in the SBC's box
7. Press the power button and the blue LED indicator will light up
    - Default OS is Windows 10 :fontawesome-brands-windows:

## BIOS Setup

### Entering BIOS

**Default**

1. Power on or reboot LattePanda
2. Press **++del++** or **++esc++** key continuously to
   enter into BIOS menu before you see the LattePanda logo on the splash screen

**Windows 10**

1. Start Menu :fontawesome-brands-windows: :material-arrow-right:
   Press **++shift++** + Restart
2. Choose :material-tools: Troubleshoot :material-arrow-right: Advanced Options
   :material-arrow-right: UEFI Firmware Settings :material-arrow-right:
   Restart

### Auto Power-on

1. Power on the LattePanda and enter the BIOS
2. BIOS Setup :material-arrow-right: Advanced :material-arrow-right:
   Power Management :material-arrow-right: ACPowerLoss :material-arrow-right:
   Set the ACPowerLoss option to `PowerOn`
3. Save & Exit (Press **++f4++**)

### Configurable TDP

1. Power on the LattePanda and enter the BIOS
2. BIOS Setup :material-arrow-right: Advanced :material-arrow-right:
   CPU Power & Performance :material-arrow-right: CPU - Power Management Control
   :material-arrow-right: Config TDP Configurations :material-arrow-right:
   Change the Configurable TDP Boot Mode option

| TDP Boot Mode | Base (W) | PL1 (W) | PL2 (W) | Ratio | TAR |
| :------------ | :------: | :-----: | :-----: | :---: | :-: |
| Nominal       | 28W      | 45W     | 64W     | 19    | 18  |
| Level 1       | 20W      | 20W     | 20W     | 14    | 13  |
| Level 2       | 3W       | 35W     | 64W     | 22    | 21  |

_(default values set in BIOS)_

- **PL:** Power Limit
- **TAR:** Turbo Activation Ratio (can be overwritten)

## Power

- **:material-usb-c-port: USB Type-C Power Delivery(PD)**
    - Power adapter provided in the box
    - UGREEN Nexode Power Bank 100W 20000mAh
      _(portable solution to have onboard the robot)_

!!! Tip

    Using a powerbank instead of using the JST PH2.0-8Pin input connector
    provides an electrically isolated power option from the robot's LiPo battery
    for powering the SBC.

## Storage

- **:fontawesome-solid-memory: Crucial P3 500 GB M.2 PCIe Gen3 NVMe SSD**
  **CT500P3SSD8** _(M.2 Key-M)_

## Display Connections

- **:material-hdmi-port: HDMI Dummy Plug**
  _(remote connection w/ graphical interface wo/ monitor)_
