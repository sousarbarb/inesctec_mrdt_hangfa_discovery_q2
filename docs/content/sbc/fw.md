# Firmware Setup

## Device Rules (udev)

### PlatformIO Devices

```sh
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart
```

Source:
[https://docs.platformio.org/en/latest/core/installation/udev-rules.html#platformio-udev-rules](https://docs.platformio.org/en/latest/core/installation/udev-rules.html#platformio-udev-rules).

### Specific Device Rules

!!! Tip

    Specific device rules allows to define a particular name for a certain
    device, allowing static naming when connection through serial port
    communication.

    In particularly with LattePanda SBCs, this feature is even more important
    to separate clearly which device is one (depending on the restart of the
    SBC with an Arduino or other device already plugged into the LattePanda,
    the onboard ATMega32 may be either `/dev/ttyACM0` or `/dev/ttyACM1`).

    Source:
    [https://roboticsknowledgebase.com/wiki/tools/udev-rules/](https://roboticsknowledgebase.com/wiki/tools/udev-rules/).

1. Open :material-microsoft-visual-studio-code: Visual Studio Code
    - GNOME menu
    - Terminal (`$ code` | `$ code .` to open the VS Code in the current
      directory)
2. Open :simple-platformio: PlatformIO extension
    - PIO Home :material-arrow-right: Devices
        - `/dev/ttyACM0`
            - Description: `LattePanda Leonardo`
            - Hardware: `USB VID:PID=3343:803A LOCATION=1-5:1.0`
        - `/dev/ttyACM1`
            - Description: `ttyACM1`
            - Hardware: `USB VID:PID=2341:0042 SER=9593132393135171D1E2 LOCATION=1-2:1.0`

    !!! Tip

        Another way to check the previous information is directly through
        terminal commands:

        ```sh
        udevadm info -a -n /dev/ttyACM0
        udevadm info -a -n /dev/ttyACM1
        ```

3. Execute the following commands

    ```sh
    sudo touch /etc/udev/rules.d/00-usb-serial.rule
    sudo tee /etc/udev/rules.d/00-usb-serial.rule <<EOF
    # LattePanda Leonardo (ATMega32 onboard of the SBC)
    SUBSYSTEM=="tty", ATTRS{idVendor}=="3343", ATTRS{idProduct}=="803a", SYMLINK+="ttyLattePandaATMega"

    # Arduino Mega
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduinoMega"

    EOF
    ```

    !!! Warning

        NOT YET CONFIRMED...


## CoolTerm

!!! Tip

    CoolTerm is a more advanced serial monitor with an User Interface (UI),
    compared to the native serial monitor implementation of PlatformIO.

1. Download CoolTerm (available at
   [https://freeware.the-meiers.org/](https://freeware.the-meiers.org/))
2. Execute the following commands
    ```sh
    mkdir ~/dev/tools -p
    mv ~/Downloads/CoolTermLinux64Bit.zip ~/dev/tools/

    cd ~/dev/tools/
    unzip CoolTermLinux64Bit.zip
    ```
3. Open CoolTerm
    ```sh
    cd ~/
    ./dev/tools/CoolTermLinux64Bit/CoolTerm
    ```
4. Connection :material-arrow-right: Options
    - Serial Port
        - Port: `/dev/ttyACM1` _(note that LattePanda SBCs comes with ATMega32)_
        - Baudrate: `115200` _(check in your code the serial port's baud rate)_
        - Data bits: `8`
        - Parity: `None`
        - Stop bits: `1`
    - Receive
        - Enable _Direct RX update of text display_
        - Enable _Add timestamps to received data_

            !!! Tip

                Enabling timestamps may be useful to check the cycle time
                period.

        - Set _Type_ to _Time + Milliseconds_
