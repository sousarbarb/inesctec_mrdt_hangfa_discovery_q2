# LattePanda 3 Delta

## Links

- [LattePanda 3 Delta](https://www.lattepanda.com/lattepanda-3-delta)
- [LattePanda 3 Delta Specification](https://docs.lattepanda.com/content/3rd_delta_edition/specification/)
- [Pinout and Hardware Diagram](https://docs.lattepanda.com/content/3rd_delta_edition/io_playability/)
- [Initial Setup of the 3 Delta by LattePanda](https://docs.lattepanda.com/content/3rd_delta_edition/get_started/)
- [LattePanda 3 Delta 3D Model](https://github.com/LattePandaTeam/LattePanda-3D-Models/tree/master/LattePanda%203%20Delta)

## Specifications

| Content             | Specs.                                                       |
| ------------------- | ------------------------------------------------------------ |
| Processor           | Intel® Celeron® Processor N5105 (Frequency: 2.00GHz ~ 2.90GHz) |
| Graphics            | Intel® UHD Graphics (Frequency: 450MHz ~ 800MHz)             |
| Memory              | 8GB LPDDR4 2933MHz                                           |
| Storage             | 64GB eMMC V5.1                                               |
| Wireless            | Intel® AX201 WiFi 6 (802.11ax), 2.4GHz and 5GHz (160MHz) Supported; Bluetooth® 5.2 |
| Ethernet            | Intel® I211-AT / I225-V PCIe Gigabit LAN, Wake-On-LAN Supported |
| Video               | HDMI 2.0b; DP 1.4 via USB Type-C; eDP 30Pin (two lanes)      |
| Audio               | Microphone + Headphone Combo Connector                       |
| USB                 | 1x USB 3.2 gen 2 Type-A; 2x USB 3.2 gen1 Type-A; 1x USB 2.0 Type-C |
| Expansion Slots     | M.2 Key B(2242/2252/2280): SATA III, USB2.0, USB3.0, SIM; M.2 Key M(2280): PCIe 3.0 x2; Micro-SD + Micro-SIM Combo Card Connector |
| Coprocessor         | Microchip® ATmega32U4-MU                                     |
| GPIO Female Headers | Atmega32U4 Pinout; BIOS Flash Pinout; 5V & 3.3V Output; 12V DC Input; USB 2.0; RS232; Audio; Status Control & Indication |
| TPM                 | Built-in TPM (2.0)                                           |
| BIOS Function       | Auto Power On, Watchdog Timer, Serial Port Redirection, etc. |
| Power               | PH2.0-4Pin DC Input: 12V; USB Type-C PD Input: 15V DC        |
| RTC                 | CR927 3V                                                     |
| Dimension           | 125mm x 78mm x16mm                                           |
| Default Built-in OS | Windows 10 Pro 21H2; Windows 10 IoT Enterprise 2021 LTSC     |
| Compatible OS       | Windows 10, Windows 11, Ubuntu                               |
| Environment         | Operating Temperature: 0~60℃; Relative Humidity: 0%~80%      |
| Certification       | CE, FCC, KC, RoHS                                            |

## Initial Setup

**Preparations**

- :material-cable-data:         HDMI Cable
- :material-monitor:            External HDMI Monitor
- :material-keyboard:           Keyboard and a Mouse
- :material-power-plug-battery: Power Adapter
- :material-antenna:            Pair of WIFI/BT antennas
    - Antennas that come in the box
    - Bingfu Antena WiFi Dual Band RP-SMA 25cm U.FL IPX IPEX MHF4 M.2 NGFF Intel

!!! Warning "WiFi Antenna Interface"

    WiFi card on the LattePanda 3 Delta is not a traditional one as
    you see in desktops (the connector is different: IPEX4 2.4 & 5G Dual-band).

**Instructions**

1. Attach the Wi-Fi/BT antennas to the sockets
2. Connect an HDMI display
3. Connect the keyboard and mouse to the LattePanda 3 Delta
4. Connect the power adapter provided in the SBC's box
5. Press the power button and the blue LED indicator will light up
    - Default OS is Windows 10 :fontawesome-brands-windows:

## BIOS Setup

### Entering BIOS

**Default**

1. Power on or reboot LattePanda
2. Press **++del++** or **++esc++** key continuously to
   enter into BIOS menu before you see the LattePanda logo on the splash screen

**Windows 10**

1. Start Menu :fontawesome-brands-windows: :material-arrow-right:
   Press Left Shift + Restart
2. Choose :material-tools: Troubleshoot :material-arrow-right: Advanced Options
   :material-arrow-right: UEFI Firmware Settings :material-arrow-right:
   Restart

### Auto Power-on

1. Power on the LattePanda and enter the BIOS
2. BIOS Setup :material-arrow-right: Advanced :material-arrow-right:
   Power Management :material-arrow-right: ACPowerLoss :material-arrow-right:
   Set the ACPowerLoss option to `PowerOn`
3. Save & Exit (Press **++f4++**)

## Power

- **:material-usb-c-port: USB Type-C Power Delivery(PD)**
    - Power adapter provided in the box
    - Powerbank Xiaomi Mi 50W 20000 mAh _(portable solution to have onboard the_
      _robot)_

!!! Tip

    Using a powerbank instead of using the JST PH2.0-4Pin input connector
    provides an electrically isolated power option from the robot's LiPo battery
    for powering the SBC.

## Storage

- **:material-memory: 64GB eMMC V5.1** _(internal)_
- **:fontawesome-solid-memory: Crucial P3 500 GB M.2 PCIe Gen3 NVMe SSD**
  **CT500P3SSD8** _(M.2 Key-M)_

## Display Connections

- **:material-hdmi-port: HDMI Dummy Plug**
  _(remote connection w/ graphical interface wo/ monitor)_
