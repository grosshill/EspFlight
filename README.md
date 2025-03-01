# EspFlight
Light weighted integrate flight control based on ESP-IDF and ESP32-S3.

Skeleton:
| 外设需求 | 优先级 | 实现 | 备注 |
|:-----|:-----|:-----|:-----|
| 陀螺仪/加速度计驱动 | 必需   | 已实现 | MPU6050 |
| 气压计驱动 | 必需   | 已实现 | BMP180 |
| PWM电调驱动 | 必需  | 未实现| |
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