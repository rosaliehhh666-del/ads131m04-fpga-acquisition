################################################################################
# daq_board_top_zhengzhan_core.xdc
# Board        : 征战 Core
# FPGA         : XC7A35T-2FGG484
# Top          : daq_board_top
# System clock : 50 MHz
#
# Architecture:
#   ADS131M04 <-> FPGA : independent SPI (FPGA master)
#   MCU       <-> FPGA : independent SPI (MCU master, FPGA slave)
#   FPGA       -> PC   : onboard CH340E UART -> VOFA+
################################################################################

# ------------------------------------------------------------------------------
# 50 MHz board clock: FPGA_GCLK -> Y18
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN Y18 [get_ports clk_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
create_clock -name clk_50m -period 20.000 -waveform {0.000 10.000} [get_ports clk_50m]

# ------------------------------------------------------------------------------
# Reset: SW1 -> H5, normally high / pressed low
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN H5 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

# ------------------------------------------------------------------------------
# ADS131M04 SPI - U15B expansion header, Bank 34
#
# Corrected header mapping from the core-board schematic:
#   U15B-103 -> T1
#   U15B-104 -> U1
#   U15B-105 -> U2
#   U15B-106 -> V2
#   U15B-107 -> W2
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN T1 [get_ports adc_drdy]
set_property IOSTANDARD LVCMOS33 [get_ports adc_drdy]

set_property PACKAGE_PIN U1 [get_ports adc_dout]
set_property IOSTANDARD LVCMOS33 [get_ports adc_dout]

set_property PACKAGE_PIN U2 [get_ports adc_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports adc_cs_n]

set_property PACKAGE_PIN V2 [get_ports adc_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports adc_sclk]

set_property PACKAGE_PIN W2 [get_ports adc_din]
set_property IOSTANDARD LVCMOS33 [get_ports adc_din]

# ------------------------------------------------------------------------------
# ADS131M04 hardware RESET / SYNC (active low)
#
# FPGA pin: P1
#
# Connect P1 to ADS131M04 pin 11 (SYNC/RESET).
# Verify with your schematic/wiring before use.
# The existing FPGA push-button reset remains SW1 -> H5.
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN P1 [get_ports adc_reset_n]
set_property IOSTANDARD LVCMOS33 [get_ports adc_reset_n]
set_property DRIVE 8 [get_ports adc_reset_n]
set_property SLEW SLOW [get_ports adc_reset_n]

set_property DRIVE 8 [get_ports {adc_cs_n adc_sclk adc_din}]
set_property SLEW SLOW [get_ports {adc_cs_n adc_sclk adc_din}]

# ------------------------------------------------------------------------------
# MCU SPI - next four U15B Bank34 GPIOs
# MCU is master; FPGA is slave; SPI mode 0; use <= 2 MHz initially.
#
#   U15B-108 -> Y2 : mcu_cs_n
#   U15B-109 -> W1 : mcu_sclk
#   U15B-110 -> Y1 : mcu_mosi
#   U15B-111 -> U3 : mcu_miso
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN Y2 [get_ports mcu_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports mcu_cs_n]

set_property PACKAGE_PIN W1 [get_ports mcu_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports mcu_sclk]

set_property PACKAGE_PIN Y1 [get_ports mcu_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports mcu_mosi]

set_property PACKAGE_PIN U3 [get_ports mcu_miso]
set_property IOSTANDARD LVCMOS33 [get_ports mcu_miso]
set_property DRIVE 8 [get_ports mcu_miso]
set_property SLEW SLOW [get_ports mcu_miso]

# MCU SPI is asynchronous to the FPGA's 50 MHz system clock and is explicitly
# synchronized in mcu_spi_slave.v. Do not time it as a synchronous 50 MHz path.
set_false_path -from [get_ports {mcu_cs_n mcu_sclk mcu_mosi}]

# ------------------------------------------------------------------------------
# On-board USB-UART CH340E
# CH340E RXD is connected to FPGA AB6_L20N_B34.
# Therefore FPGA uart_tx must be placed on AB6.
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN AB6 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property DRIVE 8 [get_ports uart_tx]
set_property SLEW SLOW [get_ports uart_tx]

# Dedicated Linux waveform TX, FPGA AB5 to 3.3 V UART RX.
set_property -dict {PACKAGE_PIN AB5 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW} [get_ports linux_tx]
set_property -dict {PACKAGE_PIN AA5 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports linux_rx]
set_false_path -from [get_ports linux_rx] -to [get_pins {u_screen_control/sync_rx_reg[0]/D}]

