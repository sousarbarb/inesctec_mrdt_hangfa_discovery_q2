# Development Environment

## Doxygen

```sh
sudo apt update
sudo apt install doxygen doxygen-doc doxygen-latex -y
```

## Visual Studio Code

### Installation

1. Download :material-microsoft-visual-studio-code: Visual Studio Code
   (available at
   [https://code.visualstudio.com/download](https://code.visualstudio.com/download))
2. Execute the following commands
    ```sh
    cd ~Downloads/
    sudo apt install ./code_1.97.2-1739406807_amd64.deb -y
    ```

### Setup

1. Open :material-microsoft-visual-studio-code: Visual Studio Code
    - GNOME menu
    - Terminal (`$ code` | `$ code .` to open the VS Code in the current
      directory)
2. :material-view-grid-plus-outline: Extensions
   (**++ctrl+shift+x++** | File :material-arrow-right: Preferences
   :material-arrow-right: Extensions)
    - :simple-cplusplus: ms-vscode.cpptools-extension-pack
        - ms-vscode.cpptools
        - ms-vscode.cpptools-themes
        - ms-vscode.cmake-tools
    - :material-language-python: ms-python.python
    - :material-language-python: ms-python.debugpy
    - :material-language-python: ms-python.vscode-pylance
    - :simple-doxygen: cschlosser.doxdocgen
    - :simple-ros: :material-robot: ms-iot.vscode-ros
    - :material-file-xml-box: :material-xml: redhat.vscode-xml
    - :simple-yaml: redhat.vscode-yaml
    - :material-file-pdf-box: tomoki1207.pdf
    - :simple-platformio: platformio.platformio-ide

        !!! Warning

            PlatformIO requires the Python Virtual Environment to be installed
            as well as possibly needing to have `python` as an alias to
            `python3`
            ([https://docs.platformio.org/en/latest/faq/install-python.html](https://docs.platformio.org/en/latest/faq/install-python.html)).

            ```sh
            sudo apt install python-is-python3 python3-venv -y
            ```

    - :material-owl: sdras.night-owl

3. User settings (**++ctrl+shift+p++** :material-arrow-right:
   _Preferences: Open User Settings (JSON)_)
    ```json title="settings.json"
    {
        "editor.renderWhitespace": "none",
        "editor.tabSize": 2,
        "files.trimTrailingWhitespace": true,
        "files.trimFinalNewlines": true,
        "editor.rulers": [
            80, 120
        ],
        "cmake.configureOnOpen": false,
        "workbench.colorTheme": "Night Owl",
        "explorer.confirmDragAndDrop": false,
        "explorer.confirmDelete": false,
        "terminal.integrated.stickyScroll.enabled": false,
        "git.openRepositoryInParentFolders": "always",
        "window.title": "${dirty}${rootName}"
    }
    ```
