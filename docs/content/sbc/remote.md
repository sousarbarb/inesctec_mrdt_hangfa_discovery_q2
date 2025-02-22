# Remote Access

## AnyDesk

1. Download AnyDesk (available at
   [https://anydesk.com/en/downloads/linux](https://anydesk.com/en/downloads/linux))
2. Execute the following commands
    ```sh
    cd ~/Downloads/
    sudo apt update
    sudo apt install ./anydesk_6.4.2-1_amd64.deb -y
    ```
3. Open AnyDesk
4. _(optional)_ Set an alias to use it for remote access instead of the
   AnyDesk ID
5. Open the :material-menu: Menu :material-arrow-right:
   :material-wrench: Settings
    - :material-lock: Security :material-arrow-right: :material-security: Unlock
        - Enable unattended access:
            - Set a password
            - Set _Full Access_ for the permission profile
        - _(optional)_ Disable _Search local network for other AnyDesk clients_
        - _(optional)_ Enable _Exclude this device from discovery_
    - _(optional)_ :fontawesome-solid-display: Display
        - Set _Best performance_ in the quality
        - Set _Optimize screen usage (strecht)_ in view mode

!!! Tip

    If you are having problems connecting to another computer through AnyDesk,
    try to remove the local cache folder.

    ```sh
    ll ~/.anydesk/
    rm -rf ~/.anydesk/
    ```

!!! Warning

    AnyDesk requires both remote access endpoints to have Internet connection.

!!! Warning

    Some IT managers may lock the remote access through AnyDesk by disabling
    connections through `*.net.anydesk.com`, even if both endpoints have an
    Internet connection.
    More information in
    [https://support.anydesk.com/knowledge/firewall](https://support.anydesk.com/knowledge/firewall).

## RustDesk

1. Download RustDesk (available at
   [https://github.com/rustdesk/rustdesk/releases/](https://github.com/rustdesk/rustdesk/releases/))
2. Execute the following commands
    ```sh
    cd ~/Downloads/
    sudo apt update
    sudo apt install ./rustdesk-1.3.7-x86_64.deb -y
    ```
3. Open RustDesk
4. Open the :material-menu: Menu :material-arrow-right:
   :material-wrench: Settings:
    - :material-lock: Security :material-arrow-right: :material-security: Unlock
      Security Settings
        - Use permanent password
        - _(optional)_ Deny LAN discovery
        - _(optional)_ Enable direct IP access :material-arrow-right: Define
          the Port (e.g., `21118`)
    - _(optional)_ :fontawesome-solid-display: Display
        - Set _Scale adaptive_
        - Set _Optimize reaction time_
        - Other default options
            - [x] Show monitors in toolbar
            - [x] Enable file copy and paste
            - [x] Use all my displays for the remote session

!!! Tip

    RustDesk is an open-source remote access software as an alternative to
    AnyDesk and similiar applications, allowing your own self-hosted servers
    and direct IP access to your machine.

!!! Warning

    Direct IP access through RustDesk remote connection is unecrypted!