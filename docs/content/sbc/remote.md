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
