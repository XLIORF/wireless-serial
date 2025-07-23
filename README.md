

# wireless-serial

`wireless-serial` 是一个基于 ESP32 平台的无线串口通信项目，主要利用 ESP-NOW 协议实现设备之间的低延迟、无需连接 Wi-Fi 网络的数据传输。该项目适用于物联网设备间的快速数据交换场景。

## 特性

- 使用 ESP-NOW 实现无线串口通信
- 支持广播和单点发送模式
- 提供 UART 接收任务处理串口数据
- 基于 FreeRTOS 的任务调度机制

## 目录结构

- `main/`：主要源码目录，包含 `espnow.c` 和 `main.c` 等核心文件
- `CMakeLists.txt`, `Makefile`, `Kconfig.projbuild`, `component.mk`：构建配置文件
- `.gitignore`：指定 Git 忽略的文件类型
- `.vscode/settings.json`：VSCode 编辑器配置文件
- `sdkconfig`, `sdkconfig.old`：ESP-IDF SDK 配置文件

## 主要模块

### `espnow.c`

提供 ESP-NOW 协议相关功能，包含以下主要函数：

- `wifi_init()`：初始化 Wi-Fi 模块
- `espnow_init()`：初始化 ESP-NOW
- `espnow_send_package()`：发送 ESP-NOW 数据包
- `espnow_task()`：ESP-NOW 处理任务
- `uart_rx_task()`：UART 接收处理任务

### `main.c`

程序入口文件，包含：

- `app_main()`：主函数，初始化系统并启动任务

## 使用说明

1. 安装 ESP-IDF 开发环境
2. 克隆项目到本地：`git clone https://gitee.com/WineKite/wireless-serial`
3. 进入项目目录并配置：`idf.py menuconfig`
4. 编译项目：`idf.py build`
5. 烧录并运行：`idf.py -p (PORT) flash monitor`

## 许可证

本项目遵循 Apache-2.0 协议。详见 LICENSE 文件。