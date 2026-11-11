# ADS131M04 四通道低频信号采集

使用 ADS131M04 同步采集四路低频信号，由 Artix-7 FPGA 完成低通滤波、抽取和串口封帧，在 Linux 上显示波形并测量同频信号的幅值比和相位差。

- 四路 24 位同步采集，工程标称每通道 16 kSPS。
- 257 抽头 FIR 低通后按 8 倍抽取，上传每通道 2 kSPS 的波形。滤波器的设计通带约为 800 Hz。
- 串口采用双缓冲，每帧包含 32 组四路样点，并附有帧序号、样点序号及 CRC32。
- Linux 显示提供 0.1、1、5、10 秒时基、DC／AC 显示、均值、交流有效值和峰峰值。输入已知频率可拟合幅相，导入实测校准系数可显示电压。

## 工程目录

| 路径 | 内容 |
|---|---|
| [fpga](fpga/) | Vivado 工程、Verilog、引脚约束与 FIR 系数 |
| [linux](linux/) | 串口接收、浏览器波形页和 [测量说明](linux/README_measurement.md) |
| [hardware](hardware/) | [嘉立创EDA原理图及 PCB 工程](hardware/four_channels_fpga.epro) |

## 使用方法

1. 使用 Vivado 2018.3 打开 fpga/four_channels_adc.xpr，目标器件为 XC7A35T-2FGG484。按需重新生成 FFT IP 输出产品，运行综合和实现，再生成 bit。
2. 用 3.3 V TTL 串口连接 FPGA 与 Linux。FPGA AB5 接 Linux RXD，Linux TXD 接 FPGA AA5，两端共地。串口参数为 2 Mbaud、8N1。
3. 在 Linux 上运行下列命令，串口设备名按实际连接替换。

```sh
python3 linux/adc_scope.py /dev/ttyUSB0
```

在屏幕浏览器打开 [本机波形页](http://127.0.0.1:8766/)。AA5 的暂停和恢复命令见 [串口控制说明](linux/AA5.md)。

## 验证状态

现有开发记录覆盖 FIR 四通道卷积对照、UART 帧校验、连续样点传输、暂停恢复和屏端幅相拟合。AA5 版本已完成布局布线。整机频带、噪声底、最小可测输入和幅相准确度以实测为准。实物照片后续补充。

## 许可

本项目原创源码和板卡设计采用 GNU General Public License v3.0 only，全文见 [LICENSE](LICENSE)。Vivado 的 FFT IP 通过工程中的 XCI 配置在本地生成，厂商工具和 IP 适用各自许可。
