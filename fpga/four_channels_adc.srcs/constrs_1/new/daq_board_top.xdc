create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 2 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL true [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list clk_50m_IBUF_BUFG]]

# probe0: dbg_adc_reset_n (1b)
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 1 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list dbg_adc_reset_n]]

# probe1: dbg_adc_drdy (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 1 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list dbg_adc_drdy]]

# probe2: dbg_adc_cs_n (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 1 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list dbg_adc_cs_n]]

# probe3: dbg_adc_sclk (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 1 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list dbg_adc_sclk]]

# probe4: dbg_adc_din (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 1 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list dbg_adc_din]]

# probe5: dbg_adc_dout (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 1 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list dbg_adc_dout]]

# probe6: adc_init_state_int (4b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
set_property port_width 4 [get_debug_ports u_ila_0/probe6]
connect_debug_port u_ila_0/probe6 [get_nets [list {adc_init_state_int[0]} {adc_init_state_int[1]} {adc_init_state_int[2]} {adc_init_state_int[3]}]]

# probe7: spi_busy_int (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
set_property port_width 1 [get_debug_ports u_ila_0/probe7]
connect_debug_port u_ila_0/probe7 [get_nets [list spi_busy_int]]

# probe8: spi_cmd_done_int (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
set_property port_width 1 [get_debug_ports u_ila_0/probe8]
connect_debug_port u_ila_0/probe8 [get_nets [list spi_cmd_done_int]]

# probe9: adc_init_done_int (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe9]
set_property port_width 1 [get_debug_ports u_ila_0/probe9]
connect_debug_port u_ila_0/probe9 [get_nets [list adc_init_done_int]]

# probe10: adc_init_error_int (1b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe10]
set_property port_width 1 [get_debug_ports u_ila_0/probe10]
connect_debug_port u_ila_0/probe10 [get_nets [list adc_init_error_int]]

# probe11: spi_rx_word0_int (24b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe11]
set_property port_width 24 [get_debug_ports u_ila_0/probe11]
connect_debug_port u_ila_0/probe11 [get_nets [list {spi_rx_word0_int[0]} {spi_rx_word0_int[1]} {spi_rx_word0_int[2]} {spi_rx_word0_int[3]} {spi_rx_word0_int[4]} {spi_rx_word0_int[5]} {spi_rx_word0_int[6]} {spi_rx_word0_int[7]} {spi_rx_word0_int[8]} {spi_rx_word0_int[9]} {spi_rx_word0_int[10]} {spi_rx_word0_int[11]} {spi_rx_word0_int[12]} {spi_rx_word0_int[13]} {spi_rx_word0_int[14]} {spi_rx_word0_int[15]} {spi_rx_word0_int[16]} {spi_rx_word0_int[17]} {spi_rx_word0_int[18]} {spi_rx_word0_int[19]} {spi_rx_word0_int[20]} {spi_rx_word0_int[21]} {spi_rx_word0_int[22]} {spi_rx_word0_int[23]}]]

# probe12: spi_tx_word0_int (24b)
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe12]
set_property port_width 24 [get_debug_ports u_ila_0/probe12]
connect_debug_port u_ila_0/probe12 [get_nets [list {spi_tx_word0_int[0]} {spi_tx_word0_int[1]} {spi_tx_word0_int[2]} {spi_tx_word0_int[3]} {spi_tx_word0_int[4]} {spi_tx_word0_int[5]} {spi_tx_word0_int[6]} {spi_tx_word0_int[7]} {spi_tx_word0_int[8]} {spi_tx_word0_int[9]} {spi_tx_word0_int[10]} {spi_tx_word0_int[11]} {spi_tx_word0_int[12]} {spi_tx_word0_int[13]} {spi_tx_word0_int[14]} {spi_tx_word0_int[15]} {spi_tx_word0_int[16]} {spi_tx_word0_int[17]} {spi_tx_word0_int[18]} {spi_tx_word0_int[19]} {spi_tx_word0_int[20]} {spi_tx_word0_int[21]} {spi_tx_word0_int[22]} {spi_tx_word0_int[23]}]]

set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_50m_IBUF_BUFG]
