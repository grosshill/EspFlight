### 2025.9.15
修复了 `esp32-S3` 在写入`bmi270` 8192字节的配置文件时 core panic的错误。
原因是i2c以400khz写入这个数据耗时大约185ms，但是在总线配置中默认值是100ms，这就会导致在time_out期限结束之前传输无法完成。解决办法是将总线time_out修改为400ms。

```
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