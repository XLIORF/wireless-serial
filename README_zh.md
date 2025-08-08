# Wireless Serial

基于 ESP-NOW 协议的无线串口通信项目，可在两个 ESP8266 设备之间建立透明的串口透传连接。

## 项目简介

Wireless Serial 是一个基于 ESP8266 和 ESP-NOW 协议实现的无线串口通信解决方案。该项目允许两个 ESP8266 设备通过 ESP-NOW 协议建立连接，并实现串口数据的无线透传。一个设备作为主设备（Master），另一个作为从设备（Slave），两者通过自动协商建立连接后，即可实现双向串口数据传输。

## 功能特性

- 基于 ESP-NOW 协议的低延迟无线通信
- 自动主从设备协商机制
- 串口透传，支持任意波特率数据传输
- 可靠的数据包校验和重传机制
- 支持连接状态检测和断线重连
- 双向数据传输

## 硬件要求

- 2个 ESP8266 开发板（如 ESP-12F、NodeMCU 等）
- USB 转串口适配器（用于烧录和调试）
- 连接电脑的串口终端软件

## 软件依赖

- ESP8266 RTOS SDK
- esptool.py（烧录工具）

## 构建和烧录

## 配置环境

确保已正确设置 ESP8266 RTOS SDK 环境：

激活环境：
```bash
. ESP8266_RTOS_SDK/export.sh 
```

### 配置日志输出

如果不需要日志输出可以关闭，操作如下
```
make menuconfig --> Bootloader config ---> Bootloader log verbosity --->  No output
make menuconfig --> Component config ---> Log output ---> Default log output ---> No output
```
这样只有连接、断开和输出被打印了。

屏蔽ROM code 日志的方法：https://www.freesion.com/article/8701586207/

状态输出如下：
```
Broadcast.
Connected.
Disconnected.
```
### 构建项目

```bash
make menuconfig  # 可选，配置项目参数
make
```
### 烧录固件

使用提供的脚本烧录两个设备：

```bash
make flash
```

或者

```bash
./two_flash.sh
```
该脚本会同时向两个 ESP8266 设备（/dev/ttyUSB0 和 /dev/ttyUSB1）烧录固件。

或者手动烧录：

```bash
make flash ESPPORT=/dev/ttyUSB0
make flash ESPPORT=/dev/ttyUSB1
```
### 使用方法
将两个 ESP8266 设备烧录固件并上电
设备会自动协商主从关系并建立连接
连接成功后，通过串口终端软件连接到两个设备的串口
在任一串口输入内容，将在另一个串口输出，实现透明传输

### 测试工具
项目包含一个 Python 测试脚本 serial_test.py，可用于测试串口通信的可靠性和性能：

```bash
python serial_test.py --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1 --baudrate 115200
```
## 项目结构
```
wireless-serial/
├── main/              # 主要源代码
│   ├── main.c         # 主程序入口
│   ├── wlcon.c        # ESP-NOW 无线连接实现
│   └── wlcon.h        # 头文件
├── Makefile           # 构建配置
├── twoflash.sh        # 双设备烧录脚本
└── serial_test.py     # 串口通信测试工具
```

## 工作原理

1. 设备启动后进入广播状态，寻找其他设备
2. 通过协商机制确定主从关系
3. 建立 ESP-NOW 连接
4. 通过 UART 接收串口数据并无线发送
5. 接收无线数据并通过 UART 发送至串口

## 故障排除

- 如果设备无法连接，请确保两个设备都在通信范围内
- 检查串口连接是否正确
- 查看日志输出以获取更多调试信息
- 确保使用相同的波特率设置

# 许可证
带上作者的名字即可，随意使用。

# 贡献
欢迎提交 Issue 和 Pull Request 来改进这个项目。