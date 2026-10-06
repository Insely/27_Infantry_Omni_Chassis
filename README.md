# 27 Infantry Chassis

目录结构与对应的 `27_Fold_Chassis` 工程一致。

```text
.doc/          说明、修改记录与联测脚本
.vscode/       VS Code 构建和调试配置
build/         本板 ELF、HEX、BIN 和中间文件
Core/          STM32 初始化、任务调度
Drivers/       HAL、CMSIS、板载芯片驱动
debug/         OpenOCD、SVD、Ozone 配置
Middlewares/   FreeRTOS、USB 协议栈
USB_DEVICE/    USB 设备配置
User/          BSP、Hardware、Algorithm、System、Software
```

配置入口：`User/System/robot_param.h`。兵种选择 `ROBOT_TYPE`，全向轮 `CHASSIS_TYPE=1`，默认板间 CAN2。通信协议位于 `User/Software/Infantry/BoardLink.c/.h`；哨兵预留在 `User/Software/Sentry`。两块板需要匹配的协议和参数。

```sh
make -j8
```

本板可独立复制、编译，输出为 `build/27_Infantry_Chassis.elf/.hex/.bin`。VS Code 打开本板工程目录后可直接使用构建任务及 STlink / DAPlink 调试配置。

软件验证与迁移细节见 [.doc/AI修改记录](.doc/AI修改记录/目录调整.md)。双板联测使用 `make test`，完整编译矩阵使用 `make check`，需要保留相邻的另一板工程。本机主机测试需指定兼容 SDK：

```sh
HOST_CFLAGS="-isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX15.5.sdk" make test
```

控制逻辑沿用上一版，未做实车验证；CubeMX 重新生成时需保留任务调度及串口配置修改。
