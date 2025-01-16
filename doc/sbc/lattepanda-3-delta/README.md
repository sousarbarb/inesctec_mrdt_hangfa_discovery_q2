# [LattePanda 3 Delta](https://www.lattepanda.com/lattepanda-3-delta)

## Links

- [LattePanda 3 Delta](https://www.lattepanda.com/lattepanda-3-delta)
- [LattePanda 3 Delta Specification](https://docs.lattepanda.com/content/3rd_delta_edition/specification/)
- [Pinout and Hardware Diagram](https://docs.lattepanda.com/content/3rd_delta_edition/io_playability/)
- [Initial Setup of the 3 Delta by LattePanda](https://docs.lattepanda.com/content/3rd_delta_edition/get_started/)
- [LattePanda 3 Delta 3D Model](https://github.com/LattePandaTeam/LattePanda-3D-Models/tree/master/LattePanda%203%20Delta)

## Documents

- [LattePanda 3 Delta 3D Model](sbc_lattepanda-3-delta_3d-model.stp)
- [Pinout Bottom (webpage)](sbc_lattepanda-3-delta_pinout_bottom.webp)
- [Pinout Top (webpage)](sbc_lattepanda-3-delta_pinout_top.webp)

## Specification

| Type                | Description                                                        |
| ------------------- | ------------------------------------------------------------       |
| Processor           | Intel® Celeron® Processor N5105 (Frequency: 2.00GHz ~ 2.90GHz)     |
| Graphics            | Intel® UHD Graphics (Frequency: 450MHz ~ 800MHz)                   |
| Memory              | 8GB LPDDR4 2933MHz                                                 |
| Storage             | 64GB eMMC V5.1                                                     |
| Wireless            | Intel® AX201 WiFi 6 (802.11ax), 2.4GHz and 5GHz (160MHz) Supported; Bluetooth® 5.2 |
| Ethernet            | Intel® I211-AT / I225-V PCIe Gigabit LAN, Wake-On-LAN Supported    |
| Video               | HDMI 2.0b; DP 1.4 via USB Type-C; eDP 30Pin (two lanes)            |
| Audio               | Microphone + Headphone Combo Connector                             |
| USB                 | 1x USB 3.2 gen 2 Type-A; 2x USB 3.2 gen1 Type-A; 1x USB 2.0 Type-C |
| Expansion Slots     | M.2 Key B(2242/2252/2280): SATA III, USB2.0, USB3.0, SIM; M.2 Key M(2280): PCIe 3.0 x2; Micro-SD + Micro-SIM Combo Card Connector |
| Coprocessor         | Microchip® ATmega32U4-MU                                           |
| GPIO Female Headers | Atmega32U4 Pinout; BIOS Flash Pinout; 5V & 3.3V Output; 12V DC Input; USB 2.0; RS232; Audio; Status Control & Indication |
| TPM                 | Built-in TPM (2.0)                                                 |
| BIOS Function       | Auto Power On, Watchdog Timer, Serial Port Redirection, etc.       |
| Power               | PH2.0-4Pin DC Input: 12V; USB Type-C PD Input: 15V DC              |
| RTC                 | CR927 3V                                                           |
| Dimension           | 125mm x 78mm x16mm                                                 |
| Default Built-in OS | Windows 10 Pro 21H2; Windows 10 IoT Enterprise 2021 LTSC           |
| Compatible OS       | Windows 10, Windows 11, Ubuntu                                     |
| Environment         | Operating Temperature: 0~60℃; Relative Humidity: 0%~80%           |
| Certification       | CE, FCC, KC, RoHS                                                  |

## Power

- **USB Type-C Power Delivery(PD):** Powerbank Xiaomi Mi 50W 20000 mAh

## Networking

- **WiFi**
  - Available wireless network at the deployment site of the robot
  - Bingfu Antena WiFi Dual Band RP-SMA 25cm U.FL IPX IPEX MHF4 M.2 NGFF Intel
    - **Note:** the WiFi card on the SBC is not a traditional one as you see in
      desktops (the connector is different...)
- **Ethernet**
  - Local Access Network (LAN) for the robot's sensors
  -  Brainboxes 5 Port Unmanaged Ethernet Switch 10/100Mbps
    - **Note:** depending on the 3D Lidar, you may require a 1 Gbps Ethernet
      switch

## Gallery

![LattePanda 3 Delta, Xiaomi Power Bank, and Ethernet Switch](sbc-and-networking.jpg)

See the 3D model of protection and fixation cases for the Xiaomi Mi 50W 20000mAh
([bottom::step](/models/v1/battery_xiaomi-powerbank-50W-20000mAh_case_bottom.step),
[bottom::f3d](/models/v1/battery_xiaomi-powerbank-50W-20000mAh_case_bottom.f3d);
[top::step](/models/v1/battery_xiaomi-powerbank-50W-20000mAh_case_top.step),
[top::f3d](/models/v1/battery_xiaomi-powerbank-50W-20000mAh_case_top.f3d))
and the LattePanda 3 Delta
([step](/models/v1/sbc_lattepanda-3-delta_case.step),
[f3d](/models/v1/sbc_lattepanda-3-delta_case.f3d)).

See the [Bill Of Materials (BOM)](/doc/bom.pdf) for more information on the
SBC's battery (power bank solution) and Ethernet switches considered in this
work.
