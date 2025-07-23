# wireless-serial

`wireless-serial` is a wireless serial communication project based on the ESP32 platform, primarily using the ESP-NOW protocol to achieve low-latency data transmission between devices without connecting to a Wi-Fi network. This project is suitable for scenarios involving fast data exchange between IoT devices.

## Features

- Implements wireless serial communication using ESP-NOW
- Supports broadcast and unicast transmission modes
- Provides a UART receive task for processing serial data
- Utilizes a task scheduling mechanism based on FreeRTOS

## Directory Structure

- `main/`: Main source code directory, containing core files such as `espnow.c` and `main.c`
- `CMakeLists.txt`, `Makefile`, `Kconfig.projbuild`, `component.mk`: Build configuration files
- `.gitignore`: Specifies file types to be ignored by Git
- `.vscode/settings.json`: VSCode editor configuration file
- `sdkconfig`, `sdkconfig.old`: ESP-IDF SDK configuration files

## Key Modules

### `espnow.c`

Provides ESP-NOW protocol-related functionality, including the following main functions:

- `wifi_init()`: Initializes the Wi-Fi module
- `espnow_init()`: Initializes ESP-NOW
- `espnow_send_package()`: Sends an ESP-NOW data packet
- `espnow_task()`: ESP-NOW processing task
- `uart_rx_task()`: UART receive processing task

### `main.c`

Program entry point, containing:

- `app_main()`: Main function that initializes the system and starts tasks

## Usage Instructions

1. Install the ESP-IDF development environment
2. Clone the project locally: `git clone https://gitee.com/WineKite/wireless-serial`
3. Enter the project directory and configure: `idf.py menuconfig`
4. Build the project: `idf.py build`
5. Flash and run: `idf.py -p (PORT) flash monitor`

## License

This project is licensed under the Apache-2.0 License. See the LICENSE file for details.