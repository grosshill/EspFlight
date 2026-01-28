### 2025.9.15
修复了 `esp32-S3` 在写入`bmi270` 8192字节的配置文件时 core panic的错误。
原因是i2c以400khz写入这个数据耗时大约185ms，但是在总线配置中默认值是100ms，这就会导致在time_out期限结束之前传输无法完成。解决办法是将总线time_out修改为400ms。

```shell
E (1299) i2c.master: I2C software timeout
E (1299) i2c.master: s_i2c_synchronous_transaction(945): I2C transaction failed
Guru Meditation Error: Core  0 panic'ed (LoadProhibited). Exception was unhandled.

Core  0 register dump:
PC      : 0x4200d902  PS      : 0x00060930  A0      : 0x8200df24  A1      : 0x3fc9c320  
xtensa-esp32s3-elf-addr2line -fiaC -e /home/crosshill/EspFlight/build/EspFlight.elf 0x4200d902: [Errno 2] No such file or directory: 'xtensa-esp32s3-elf-addr2line'
A2      : 0x00000000  A3      : 0x00000000  A4      : 0x00000000  A5      : 0x00000513  
A6      : 0x3c026fd8  A7      : 0x3c02a77c  A8      : 0x803807f0  A9      : 0x3fc9c300  
A10     : 0x00000001  A11     : 0x3c026fd8  A12     : 0x3c0271dc  A13     : 0x3fc9c360  
A14     : 0x3fc9c340  A15     : 0x0000000c  SAR     : 0x00000004  EXCCAUSE: 0x0000001c  
EXCVADDR: 0x00000000  LBEG    : 0x400556d5  LEND    : 0x400556e5  LCOUNT  : 0xfffffff8  
xtensa-esp32s3-elf-addr2line -fiaC -e /home/crosshill/.espressif/tools/esp-rom-elfs/20241011/esp32s3_rev0_rom.elf 0x400556d5 0x400556e5: [Errno 2] No such file or directory: 'xtensa-esp32s3-elf-addr2line'


Backtrace: 0x4200d8ff:0x3fc9c320 0x4200df21:0x3fc9c380 0x4200eac2:0x3fc9c3b0 0x4200eb9a:0x3fc9c460 0x4200a425:0x3fc9c4a0 0x4200a266:0x3fc9e4d0 0x4200a0f7:0x3fc9e500 0x4037aaad:0x3fc9e530
xtensa-esp32s3-elf-addr2line -fiaC -e /home/crosshill/EspFlight/build/EspFlight.elf 0x4200d8ff 0x4200df21 0x4200eac2 0x4200eb9a 0x4200a425 0x4200a266 0x4200a0f7 0x4037aaad: [Errno 2] No such file or directory: 'xtensa-esp32s3-elf-addr2line'
```

### 2025.9.26
解决了vscode clangd和cpp扩展冲突的问题，解决之后代码补全和错误波形曲线均正常，解决方法是修改 `.vscode/settings.json` 文件
```json
{
    "C_Cpp.intelliSenseEngine": "default",
    "clangd.enable": true,
    "clangd.arguments": [
        "--query-driver=/home/crosshill/.espressif/tools/xtensa-esp32-elf/esp-2021r2-patch3/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc",
        "-I/home/crosshill/esp/esp-idf/components",
        "-I/home/crosshill/EspFlight/build/config",
        "-DESP_PLATFORM",
        "-D__XTENSA__"
    ],
    "C_Cpp.autocomplete": "default",
    "idf.espIdfPathWin": "e:\\ESP-IDF\\v5.4\\esp-idf",
    "idf.pythonInstallPath": "/home/crosshill/miniconda3/bin/python",
    "idf.openOcdConfigs": [
        "board/esp32s3-builtin.cfg"
    ],
    "idf.portWin": "COM12",
    "idf.toolsPathWin": "C:\\Users\\crosshill\\.espressif",
    "files.associations": {
        "lv_demo_widgets.h": "c",
        "cstdint": "cpp",
        "esp_err.h": "c",
        "i2c_manager.h": "c",
        "esp_log.h": "c",
        "debug_utils.h": "c",
        "array": "c",
        "string": "c",
        "string_view": "c",
        "span": "c",
        "iterator": "c",
        "vector": "c",
        "bmi270_config.h": "c",
        "freertos.h": "c",
        "format": "cpp",
        "quaternion_utils.h": "c",
        "stddef.h": "c",
        "sensor_data_type.h": "c",
        "stdint.h": "c",
        "generic_utils.h": "c",
        "time_manager.h": "c"
    },
    "idf.flashType": "UART",
    "idf.customExtraVars": {
        "IDF_TARGET": "esp32s3"
    },
    "idf.port": "/dev/ttyACM0"
}
```
注意文件中添加的 `clangd.arguments`。

### usbipd使用方法
```shell
usbipd list

Connected:
BUSID  VID:PID    DEVICE                                                         STATE
1-3    258a:00c1  USB 输入设备                                                   Not shared
1-4    303a:1001  USB 串行设备 (COM13), USB JTAG/serial debug unit               Shared
1-8    0408:30c0  USB2.0 HD UVC WebCam, USB2.0 IR UVC WebCam, Camera DFU De...  Shared
1-9    0b05:19b6  USB 输入设备                                                   Not shared
1-10   8087:0033  英特尔(R) 无线 Bluetooth(R)                                    Not shared

usbipd bind --busid 1-4
usbipd attach --wsl --busid 1-4
```
之后听到设备断连的声音就是映射成功了。


### 2026.1.25
esp32 bmi270 测速
解算 1200hz
row  1600hz

提速方案：

1. 换TDK公司的IMU，使用SPI总线以及DMA技术
2. 使用SPI总线
3. 使用FIFO
4. 摆大烂先将就用 √

#### 关于FIFO
直接 burst read mode读取fifo寄存器即可，其不会自增而是直接按照读取长度从fifo memory 里面从末尾开始顺序读取数据。
详见BMI270英文手册93页寄存器0x26 FIFO_DATA的描述


### 2026.1.28
Eigen warpper后的不透明指针单独使用时必须使用专用工厂函数和析构函数管理内存
如果嵌套在结构体里使用，则需要层次化构造/析构，建议调试好之后封装成函数进行自动内存管理