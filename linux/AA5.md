# ADS131M04 AA5 波形上传控制

接线使用 3.3 V TTL。FPGA AB5 接 Linux RXD，Linux TXD 接 FPGA AA5，两端共地。串口 2 Mbaud、8N1。AA5 启用上拉，复位后默认持续上传。屏端仍可用现有 adc_scope.py 解码四通道数据。

命令为四字节十六进制 A5 70 ENABLE CHECK。ENABLE 为 00 或 01，CHECK 为 ENABLE 按位取反。字节间超时约 1 ms，请一次写入完整命令。

| 命令 | 行为 |
|---|---|
| A5 70 00 FF | 发完当前波形帧后暂停。丢弃尚未发送的缓存，ADC 与 MCU 链路继续运行 |
| A5 70 01 FE | 重新采集一整帧后上传，样点序号保留暂停期间的间隔 |

Linux 已将串口设为 2 Mbaud 8N1 后，可以从命令行测试。将设备路径改成实际串口。

    printf '\245\160\000\377' > /dev/ttyUSB0
    printf '\245\160\001\376' > /dev/ttyUSB0

接口不单独发送确认包。验收用 AB5 上的 CRC 正确波形帧确认暂停和恢复。屏幕尚未添加按钮。命令控制上传使能，ADC 增益、采样率和 SPI 时序沿用原配置。
