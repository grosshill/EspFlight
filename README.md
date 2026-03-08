# EspFlight
Light weighted integrate flight control based on ESP-IDF and ESP32-S3.

Skeleton:
| 外设需求 | 优先级 | 实现 | 备注 |
|:-----|:-----|:-----|:-----|
| 陀螺仪/加速度计驱动 | 必需   | 已实现 | BMI270 |
| 气压计驱动 | 必需   | 已实现 | BMP280 |
| PWM电调驱动 | 必需  | 已实现| |
| Dshot电调驱动 | 可选   | 未实现 ||
| 磁罗盘/GPS驱动 | 必需   | 未实现 ||
| 测距仪/光流驱动 | 可选   | 未实现 ||
| 内置集成蓝牙接收机驱动 | 必需   | 未实现 ||
| WIFI图传 | 可选   | 未实现 ||
| 图形化地面站 | 可选   | 未实现 ||
| 外置接收机驱动(SBUS/CRSF) | 可选   | 未实现 ||

| 算法需求 | 优先级 | 实现 |
|:-----|:-----|:-----|
| 多旋翼自稳算法 | 必需   | 未实现 |
| 固定翼自稳算法 | 必需   | 未实现 |
| 多旋翼手动算法 | 可选   | 未实现 |

>文件结构:

```
EspFlight/
│── CMakeLists.txt
│── main/
│   ├── CMakeLists.txt
│   ├── main.c
│── components/
│   ├── algorithm/
│   │   ├── CMakeLists.txt
│   │   ├── pid.c
│   │   ├── include/
│   │   │   ├── pid.h
|   |   |   ...
│   ├── drivers/
│   │   ├── CMakeLists.txt
│   │   ├── i2c.c
│   │   ├── include/
│   │   │   ├── i2c.h
|   |   |   ...
│── sdkconfig

```

## TODO
Low pass FIR filter for ddx, ddy, ddz (ax, ay, az)
Low pass IIR filter for droll 
Kalman filter for postion and velocity
Cascade PID controller for postion, velocity and motor

> sensors must be collabrated before using, for barometer, check base pressure, for imu, check normalized g

> controllers should be added

>additional filter, fuse baro z and acc z

> bmi270 might need drived by SPI bus to increase work frequency, or FIFO memory is needed, currently zhe control frequency is limited up to 1.2Khz, which is obviously lower than commerical FCS like BetaFlight, which is 8Khz

## GitHub 使用指南

### Watch / Unwatch（关注 / 取消关注）

在 GitHub 仓库页面右上角有一个 **Watch** 按钮。

- **Watch（关注）**：点击后，当该仓库发生任何动态（如有新的 Issue、Pull Request、代码提交、发布版本等）时，GitHub 会向你发送通知（邮件或站内消息）。适合希望实时跟进项目进展的用户。
- **Unwatch（取消关注）**：点击后停止接收该仓库的通知。如果你不想被频繁打扰，可以选择 Unwatch，或通过下拉菜单选择只关注特定类型的通知（如仅关注自己参与的讨论）。

### Pull Request（拉取请求）

**Pull Request**（简称 PR）是 GitHub 上协作开发的核心功能。

- 当你 Fork（复制）了一个仓库并在自己的分支上做了修改，希望将这些修改合并回原仓库时，就可以发起一个 Pull Request。
- PR 相当于向仓库维护者提出"请把我的代码改动合并进来"的申请。
- 维护者可以在 PR 中查看代码差异（diff）、进行代码审查（Code Review）、留下评论，最终决定是否合并（Merge）。
- PR 也常用于团队内部，在同一仓库的不同分支之间进行代码合并与审查。

