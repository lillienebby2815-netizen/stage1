# firmware

本目录存放C板固件和嵌入式工程。

建议按功能拆分模块，例如：

- `buzzer_task.cpp/.hpp`
- `led_task.cpp/.hpp`
- `imu_task.cpp/.hpp`
- `dt7_task.cpp/.hpp`
- `motor_feedback.cpp/.hpp`
- `yaw_linkage_controller.cpp/.hpp`

实际目录结构应以队内现有工程和芯片工程生成方式为准。不要擅自移动或覆盖自动生成的底层文件。
