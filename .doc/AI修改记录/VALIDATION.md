# 验证记录

验证日期：2026-10-05；ARM 工具链：arm-none-eabi-gcc 14.3.1 / Arm GNU Toolchain 14.3.rel1；主机回放使用 macOS Clang，显式选用 MacOSX15.5 SDK（本机默认 SDK/链接器版本不匹配）。

| 检查 | 结果 |
| --- | --- |
| Gimbal / Chassis × Infantry / Sentry × CAN / RS485 / UART | 12/12 编译、链接、生成 ELF/HEX/BIN 成功 |
| 默认配置最终构建 | 双板通过，固件位于各板 `build/`；本次目录调整记录见 `目录调整.md` |
| 实际 BoardLink.c 双向收发 | 三种传输通过；KEY/小陀螺/单发/摩擦轮/超电/UI/裁判字段正确 |
| 协议异常与时序 | CRC 错误、非法枚举、NaN/Inf、TLV 边界、分包/粘包、端口隔离、过期、时钟回绕通过 |
| DMA / RS485 | DMA 忙不改写缓冲区、启动失败后重试、从机无请求不发送通过 |
| 全向轮旧/新 C 回放 | 2,000 步轮速、PID/力矩前馈电流、斜坡、四模式、LOCK 输出一致 |
| 正逆运动学 | 2,000 组 FK/IK 往返通过 |
| Yaw 旧/新 C 回放 | 2,000 步 PID、角度跨圈、遥控/自瞄切换、底盘角速度补偿一致 |
| ELF 符号分工 | Gimbal 含 YawControl、不含底盘解算；Chassis 含底盘解算、不含遥控解释；六组 Sentry 不含执行器调用入口 |

控制回放直接编译仓库的控制函数，只有 HAL/传感器/电机边界用测试输入替代；原 Chassis 代码存于 `.doc/AI修改记录/tests/reference`，便于今后继续比较。协议测试同样直接编译 `User/Software/Infantry/BoardLink.c`，不是独立重写协议的模拟器。

原始源码及参考文件哈希在 `.doc/AI修改记录/source_manifest.json`。每项编译日志和测试输出位于 `.doc/AI修改记录/validation`。上游未使用变量/宏重定义、可选 Hipnuc 文本格式、AHRS 预编译库 wchar_t、链接脚本 RWX 警告仍记录在日志中；这些检查不是零警告承诺。

没有连接实车。没有验证机械轴向、PID 真实时延、串口半双工电气时序、上电 Pitch 纠偏、发射热量保护（原有效代码未启用）、视觉端兼容性及持续运行稳定性。Pitch 反馈索引与退弹枚举修正属于本次明确记录的行为修复，不能由底盘/Yaw 回放结果推导为它们已完成实车验收。
