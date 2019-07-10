set_property -dict {PACKAGE_PIN N11 IOSTANDARD LVCMOS33} [get_ports clk_50mhz]

set_property -dict {PACKAGE_PIN E6 IOSTANDARD LVCMOS33} [get_ports led]

set_property -dict {PACKAGE_PIN N13 IOSTANDARD LVCMOS33} [get_ports {en} ]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports {sck} ]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33} [get_ports {cs123} ]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports {sdi} ]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {sdo} ]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {dir[2]} ]
set_property -dict {PACKAGE_PIN R13 IOSTANDARD LVCMOS33} [get_ports {step[2]} ]
set_property -dict {PACKAGE_PIN R12 IOSTANDARD LVCMOS33} [get_ports {dir[1]} ]
set_property -dict {PACKAGE_PIN N12 IOSTANDARD LVCMOS33} [get_ports {step[1]} ]
set_property -dict {PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports {cs456} ]
set_property -dict {PACKAGE_PIN M12 IOSTANDARD LVCMOS33} [get_ports {dir[0]} ]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {step[0]} ]
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports {dir[3]} ]
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {step[3]} ]
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports {dir[4]} ]
set_property -dict {PACKAGE_PIN P13 IOSTANDARD LVCMOS33} [get_ports {step[4]} ]
set_property -dict {PACKAGE_PIN T13 IOSTANDARD LVCMOS33} [get_ports {dir[5]} ]
set_property -dict {PACKAGE_PIN T12 IOSTANDARD LVCMOS33} [get_ports {step[5]} ]

set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {fan_hotend} ]
set_property -dict {PACKAGE_PIN R11 IOSTANDARD LVCMOS33} [get_ports {fan_part} ]
set_property -dict {PACKAGE_PIN T9 IOSTANDARD LVCMOS33} [get_ports {hotend} ]
set_property -dict {PACKAGE_PIN R10 IOSTANDARD LVCMOS33} [get_ports {bed} ]
set_property -dict {PACKAGE_PIN T8 IOSTANDARD LVCMOS33} [get_ports {probe_servo} ]
set_property -dict {PACKAGE_PIN R8 IOSTANDARD LVCMOS33} [get_ports {probe} ]
set_property -dict {PACKAGE_PIN T7 IOSTANDARD LVCMOS33} [get_ports {fan_extruder} ]

set_property -dict {PACKAGE_PIN M5 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[0]} ]
set_property -dict {PACKAGE_PIN N4 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[1]} ]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[2]} ]
set_property -dict {PACKAGE_PIN R1 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[3]} ]
set_property -dict {PACKAGE_PIN R3 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[4]} ]
set_property -dict {PACKAGE_PIN T2 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[5]} ]
set_property -dict {PACKAGE_PIN T4 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[6]} ]
set_property -dict {PACKAGE_PIN T3 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports {endstop[7]} ]

set_property -dict {PACKAGE_PIN N1 IOSTANDARD LVCMOS33} [get_ports leds_cs]
set_property -dict {PACKAGE_PIN M2 IOSTANDARD LVCMOS33} [get_ports leds_out]
set_property -dict {PACKAGE_PIN M1 IOSTANDARD LVCMOS33} [get_ports leds_clk]

set_property -dict {PACKAGE_PIN P5 IOSTANDARD LVCMOS33} [get_ports {rx} ]
set_property -dict {PACKAGE_PIN R6 IOSTANDARD LVCMOS33} [get_ports {tx} ]
set_property -dict {PACKAGE_PIN N6 IOSTANDARD LVCMOS33} [get_ports {cts} ]

create_clock -period 20.000 [get_ports clk_50mhz]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 3 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]