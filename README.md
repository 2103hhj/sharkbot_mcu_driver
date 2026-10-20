# leap_ros_mcu_driver

## 当前默认：仅电机控制（Windows 调试）

默认开启 `SHARKBOT_MOTOR_ONLY`，仅编译双电机 PWM、编码器、PID、运动控制、PID 参数读取和串口调试控制台。
Wi-Fi、micro-ROS、MAVLink、IMU、雷达、超声波、电池、灯、按键、舵机及摄像头代码已通过条件编译暂时禁用，原文件仍保留。
电机模式关闭组件管理器，不需要下载托管组件，也不依赖缺失的 `micro_ros_espidf_component`。

### Windows 编译、烧录、串口调试

使用 ESP-IDF **5.5.3**（本机已有安装），目标为 **ESP32-S3 / 4 MB Flash**。普通 PowerShell 在工程根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/idf.ps1 -Action build
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/idf.ps1 -Action flash -Port COM5
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/idf.ps1 -Action monitor -Port COM5
```

将 `COM5` 换成设备管理器中的实际串口。脚本自动读取 EIM 已安装环境（默认 `C:\Espressif\tools`，可用 `-ToolsPath` 指定），
也可从已激活的 ESP-IDF PowerShell 中运行。`ExecutionPolicy Bypass` 仅对本次进程生效。
输出目录为 `build_windows/`，不复用旧 Linux 构建缓存。脚本还支持 `menuconfig`、`size` 和 `reconfigure`。

VS Code：`Ctrl+Shift+B` 构建；“终端 → 运行任务”选择 `Motor: Flash (Windows)` 或 `Motor: Monitor (Windows)`。
这些任务自动加载环境；本机 ESP-IDF 插件的构建路径也已改为 `build_windows`。
`.clangd` 同样使用此目录的编译数据库；在其他电脑使用 ESP-IDF 插件按钮前，先选择该电脑的 Windows 安装。

默认交互串口为 **UART0，230400 baud，8N1**：ESP32 TX=`GPIO43`、RX=`GPIO44`。
使用板载 USB 转串口，或连接 3.3 V USB-UART（TX/RX 交叉、共地）。
原生 USB Serial/JTAG 若只是副控制台，仅能看日志；要通过它输入命令，需在 `menuconfig → Component config → ESP System Settings → Channel for console output`
中将其设为主控制台，并重新编译烧录。默认输出 FireWater 曲线数据；发送 `vofa 0` 后可输入 `help` 查看命令。
命令接收不显示提示符或字符回显，支持 CR、LF、CRLF 行结束；`Ctrl+]` 退出 IDF Monitor。

### VOFA+ / FireWater 速度曲线

烧录后关闭 IDF Monitor，使用 VOFA+ 独占同一个串口，设置 **230400 baud、8N1、FireWater**，添加波形控件并绑定通道。
固件默认每 **20 ms（50 Hz）** 发送一行：

```text
speed:50.123,49.876
```

| 通道 | 含义 | 单位 |
| --- | --- | --- |
| CH0 | 左轮实际速度（编码器计算并滤波） | mm/s |
| CH1 | 右轮实际速度（编码器计算并滤波） | mm/s |

帧尾是实际换行符，不是字面量 `\n`。格式遵循 [VOFA+ 官方 FireWater 文档](https://www.vofa.plus/docs/learning/dataengines/firewater/)。
数据由独立低优先级任务读取运动状态队列并发送，串口发送不阻塞电机控制任务。
曲线模式会关闭普通 ESP 日志和命令成功回显；如需查看日志、帮助和完整状态，先发送 `vofa 0`，发送 `vofa 1` 恢复曲线。

VOFA+ 发送区选择文本发送并添加换行。可发送 `motor 50 50`、`stop`、`pid 1 6 0`；
查看持续速度响应时，将 `motor 50 50` 配置为每 **500 ms** 循环发送（每条命令均加换行）。
未重复发送时，原有 **2 秒超时停车** 仍然生效，曲线会显示加速后停车。
VOFA+ 与 IDF Monitor 不能同时打开同一 COM 口。

### 电机调试命令

上电时保持停止。先将车轮悬空，发送低速命令确认接线和编码器方向。

```text
vofa 0
help
motor 50 50
status
motor 50 -50
stop
pid 1 6 0
```

| 命令 | 作用 |
| --- | --- |
| `motor <左轮速度> <右轮速度>` | 闭环目标速度，单位 mm/s，各轮范围 -500～500 |
| `stop` | 请求双轮刹车，由运动任务在下一个控制周期执行 |
| `status` | 查看实际轮速、底盘速度及编码器里程计 |
| `pid <kp> <ki> <kd>` | 更新速度环 PID，各参数 0～1000；仅本次运行有效 |
| `vofa 0` / `vofa 1` | 关闭 / 开启 FireWater 速度曲线输出，默认开启 |

**最后一条运动命令超过 2 秒未更新会自动停车**；持续运动需在 2 秒内重复发送 `motor` 命令。
`status`、`pid`、`help` 不会延长运动时间。非法数值、NaN、无穷大和越界输入会被拒绝。
运动控制周期为 20 ms；电机模式不启动 IMU，航向由左右编码器推算。
启动时仍会读取历史保存的速度 PID；不存在时使用 `kp=1、ki=6、kd=0`。

### 目前保留的接线与参数

| 项目 | 当前值 |
| --- | --- |
| 左电机 IN1 / IN2 | GPIO16 / GPIO15 |
| 右电机 IN1 / IN2 | GPIO46 / GPIO9 |
| 左编码器 A / B | GPIO18 / GPIO17 |
| 右编码器 A / B | GPIO11 / GPIO10 |
| 轮径 / 轮距 | 65 mm / 175 mm |
| 编码器每轮转计数 | 4680（四倍频后的计数） |

接线在 `main/platform/board.cpp`，机械参数与编码器方向在 `main/control/motion_controller.cpp`。
这些值保持现有源码配置，请按实际电机、减速比和安装方向校准。

### 恢复完整功能

在已激活的 ESP-IDF 环境中，用独立目录执行 `idf.py -B build_full -DSHARKBOT_MOTOR_ONLY=OFF build`。
这会恢复原有源码清单、外设及任务，并启用原组件依赖；需要先补齐 `main/idf_component.yml` 中原有的 micro-ROS 本地组件及相关依赖，
并恢复 `sdkconfig.defaults` 末尾注释的网络配置。完整固件的 micro-ROS 构建流程不属于本次 Windows 电机调试验证范围。

## 原完整固件说明（当前暂时禁用）

软件版本：`v1.5`

`leap_ros_mcu_driver` 是 Leap_ROS 底盘的 ESP32-S3 下位机驱动固件，用于电机控制、里程计、IMU、雷达、超声波、电池状态、状态灯和外设数据采集。固件支持 micro-ROS 与 MAVLink 通信模式，并提供 Wi-Fi 配网页用于运行参数配置。

## v1.5 更新内容

- 支持通过服务通讯修改速度 PID 参数，并提供获取 PID 参数服务。
- 超声波改为独立 HC-SR04，TRIG 使用 `GPIO21`，ECHO 使用 `GPIO47`。
- micro-ROS 新增 `/ultrasonic` 话题，发布超声波距离数据。
- 支持 MAVLink UDP、MAVLink UART 与 micro-ROS 通信模式切换。
- 支持长按 BOOT 键清除 Wi-Fi 配置，并重启回到配网模式。

## 底盘参数

- 当前源码 `kTrackWidth` 为 `175 mm`（历史注释为 `131.7 mm`），源码位置：`main/control/motion_controller.cpp`。

## 协议说明

MAVLink 与 micro-ROS 的消息、话题和参数说明见 [doc/protocols.md](doc/protocols.md)。
