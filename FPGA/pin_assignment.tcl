# pin_assignment.tcl
# Quartus II 引脚分配脚本 for EP4CE10F17C8
# 在 Quartus II 中执行: Tools -> Tcl Scripts -> Run

package require ::quartus::project

set_global_assignment -name FAMILY "Cyclone IV E"
set_global_assignment -name DEVICE EP4CE10F17C8
set_global_assignment -name TOP_LEVEL_ENTITY top

# -------------------------------------------------------
# 时钟
# -------------------------------------------------------
set_location_assignment PIN_E1  -to CLK_IN
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to CLK_IN

# -------------------------------------------------------
# SPI 接口
#   STM32 SPI2: SCK=PB13  NSS=PB12  MOSI=PB15  MISO=PB14
#   FPGA:       A12=MOSI  A11=MISO  (SCK/NSS 自行接线)
# -------------------------------------------------------
set_location_assignment PIN_A12 -to SPI_MOSI
set_location_assignment PIN_A11 -to SPI_MISO
set_location_assignment PIN_B11 -to SPI_SCK    ;# 根据实际飞线修改
set_location_assignment PIN_B12 -to SPI_NSS    ;# 根据实际飞线修改

set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SPI_MOSI
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SPI_MISO
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SPI_SCK
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SPI_NSS

# SPI_MISO 输出使能
set_instance_assignment -name CURRENT_STRENGTH_NEW "8MA" -to SPI_MISO

# -------------------------------------------------------
# 调试 LED (可选, 接 FPGA 开发板板载 LED)
# -------------------------------------------------------
set_location_assignment PIN_L3  -to LED[0]
set_location_assignment PIN_B1  -to LED[1]
set_location_assignment PIN_F3  -to LED[2]
set_location_assignment PIN_D1  -to LED[3]
set_location_assignment PIN_A11 -to LED[4]
set_location_assignment PIN_B13 -to LED[5]
set_location_assignment PIN_A13 -to LED[6]
set_location_assignment PIN_B14 -to LED[7]

foreach i {0 1 2 3 4 5 6 7} {
    set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to LED[$i]
}

# -------------------------------------------------------
# 时序约束
# -------------------------------------------------------
# 系统时钟 50 MHz
create_clock -name CLK_IN -period 20.0 [get_ports CLK_IN]

# SPI 时钟 (STM32 SPI2 / 8 = 168/8/8 ≈ 2.625 MHz)
create_clock -name SPI_SCK -period 381.0 [get_ports SPI_SCK]

# 异步时钟域 (SPI 相对系统时钟异步)
set_clock_groups -asynchronous \
    -group [get_clocks CLK_IN] \
    -group [get_clocks SPI_SCK]

export_assignments
