
cmake_minimum_required(VERSION 3.20)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TOOLCHAIN_FILE arm-gcc-toolchain.cmake)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_NO_VERBOSE)
set(CMAKE_C_STANDARD 11)

project(HGA22_RNDIS C ASM)






enable_language(C ASM)

set(MCU_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
)

set(COMMON_FLAGS
    -Os
    -ffunction-sections
    -fdata-sections
    -Wall
)

add_compile_options(
    ${MCU_FLAGS}
    ${COMMON_FLAGS}
    -std=gnu11
)

add_compile_definitions(
    USE_HAL_DRIVER
    STM32C071xx
    USE_FULL_LL_DRIVER
    HSE_VALUE=48000000
    HSE_STARTUP_TIMEOUT=100
    LSE_STARTUP_TIMEOUT=5000
    LSE_VALUE=32768
    EXTERNAL_CLOCK_VALUE=12288000
    LSI_VALUE=32000
    DATA_CACHE_ENABLE=1
    CFG_TUSB_MCU=OPT_MCU_STM32C0
    CFG_EXAMPLE_VIDEO_READONLY
    USE_ECM=1
    ARM_MATH_CM0PLUS=1
)

MACRO(HEADER_DIRECTORIES return_list)
    file(GLOB_RECURSE new_list LIST_DIRECTORIES true "*")
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
        #message (STATUS "dirpath : "${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

HEADER_DIRECTORIES(header_list)

#include_directories(${header_list})
include_directories(
Sources
Includes
Includes/arch
Drivers
Drivers/BSP
Drivers
Drivers/BSP/Components
Drivers/BSP/Components
Drivers/CMSIS/Core/Include
Drivers/CMSIS/Device/ST/STM32C0xx/Include
Drivers/CMSIS/DSP/PrivateInclude
Drivers/CMSIS/DSP/Include
Drivers/STM32C0xx_HAL_Driver/Inc
Drivers/STM32C0xx_HAL_Driver/Inc/Legacy
Drivers/CMSIS/Device/ST/STM32C0xx/Include
Drivers/CMSIS/Include
Drivers/BSP/STM32C0xx_Nucleo
Drivers/tinyusb/src
Drivers/tinyusb/lib/networking
Drivers/tinyusb/lib/lwip/src/include
Drivers/tinyusb/hw
Drivers/tinyusb/lib/lwip/src/include/lwip/apps
Drivers/tinyusb/lib/lwip/src/apps/http
)

file(GLOB_RECURSE CORE_SOURCES
    Sources/*.c
)




file(GLOB DRIVERS_SOURCES
     Drivers/STM32C0xx_HAL_Driver/Src/*.c
     Drivers/BSP/STM32C0xx_Nucleo/*.c
     Drivers/CMSIS/DSP/Source/BasicMathFunctions/*.c
     Drivers/CMSIS/DSP/Source/CommonTables/*.c
     Drivers/CMSIS/DSP/Source/ComplexMathFunctions/*.c
     Drivers/CMSIS/DSP/Source/FastMathFunctions/*.c
     Drivers/CMSIS/DSP/Source/TransformFunctions/*.c
     Drivers/tinyusb/typec/usbc.c
     Drivers/tinyusb/portable/st/typec/typec_stm32.c
     Drivers/tinyusb/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
     Drivers/tinyusb/portable/st/stm32_fsdev/fsdev_common.c
     Drivers/tinyusb/portable/st/stm32_fsdev/hcd_stm32_fsdev.c
     Drivers/tinyusb/networking/dhserver.c
     Drivers/tinyusb/networking/dnserver.c
     Drivers/tinyusb/networking/rndis_reports.c
     Drivers/tinyusb/netif/ethernet.c
     Drivers/tinyusb/netif/slipif.c
     Drivers/tinyusb/host/hub.c
     Drivers/tinyusb/host/usbh.c
     Drivers/tinyusb/device/usbd.c
     Drivers/tinyusb/device/usbd_control.c
     Drivers/tinyusb/core/ipv6/dhcp6.c
     Drivers/tinyusb/core/ipv6/ethip6.c
     Drivers/tinyusb/core/ipv6/icmp6.c
     Drivers/tinyusb/core/ipv6/inet6.c
     Drivers/tinyusb/core/ipv6/ip6.c
     Drivers/tinyusb/core/ipv6/ip6_addr.c
     Drivers/tinyusb/core/ipv6/ip6_frag.c
     Drivers/tinyusb/core/ipv6/mld6.c
     Drivers/tinyusb/core/ipv6/nd6.c
     Drivers/tinyusb/core/ipv4/acd.c
     Drivers/tinyusb/core/ipv4/autoip.c
     Drivers/tinyusb/core/ipv4/dhcp.c
     Drivers/tinyusb/core/ipv4/etharp.c
     Drivers/tinyusb/core/ipv4/icmp.c
     Drivers/tinyusb/core/ipv4/igmp.c
     Drivers/tinyusb/core/ipv4/ip4.c
     Drivers/tinyusb/core/ipv4/ip4_addr.c
     Drivers/tinyusb/core/ipv4/ip4_frag.c
     Drivers/tinyusb/core/altcp.c
     Drivers/tinyusb/core/altcp_alloc.c
     Drivers/tinyusb/core/altcp_tcp.c
     Drivers/tinyusb/core/def.c
     Drivers/tinyusb/core/dns.c
     Drivers/tinyusb/core/inet_chksum.c
     Drivers/tinyusb/core/init.c
     Drivers/tinyusb/core/ip.c
     Drivers/tinyusb/core/mem.c
     Drivers/tinyusb/core/memp.c
     Drivers/tinyusb/core/netif.c
     Drivers/tinyusb/core/pbuf.c
     Drivers/tinyusb/core/raw.c
     Drivers/tinyusb/core/stats.c
     Drivers/tinyusb/core/sys.c
     Drivers/tinyusb/core/tcp.c
     Drivers/tinyusb/core/tcp_in.c
     Drivers/tinyusb/core/tcp_out.c
     Drivers/tinyusb/core/timeouts.c
     Drivers/tinyusb/core/udp.c
     Drivers/tinyusb/common/tusb_fifo.c
     Drivers/tinyusb/class/video/video_device.c
     Drivers/tinyusb/class/vendor/vendor_device.c
     Drivers/tinyusb/class/vendor/vendor_host.c
     Drivers/tinyusb/class/usbtmc/usbtmc_device.c
     Drivers/tinyusb/class/net/ecm_rndis_device.c
     Drivers/tinyusb/class/net/ncm_device.c
     Drivers/tinyusb/class/mtp/mtp_device.c
     Drivers/tinyusb/class/msc/msc_device.c
     Drivers/tinyusb/class/msc/msc_host.c
     Drivers/tinyusb/class/midi/midi_device.c
     Drivers/tinyusb/apps/lwiperf/lwiperf.c
     Drivers/tinyusb/apps/http/fs.c
     Drivers/tinyusb/apps/http/httpd.c
     Drivers/tinyusb/tusb.c
     #Drivers/tinyusb/lib/lwip/src/apps/http/fsdata.c

)


set(STARTUP
    Gcc/startup_stm32c071rbtx.s
)

add_executable(${PROJECT_NAME}.elf
    ${CORE_SOURCES}
    ${DRIVERS_SOURCES}
    ${STARTUP}
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Gcc/STM32C071RBTX_FLASH.ld)

#target_link_options(${PROJECT_NAME}.elf PRIVATE
#    ${MCU_FLAGS}
#    -T "${LINKER_SCRIPT}"
#    -Wl,--gc-sections
#    --specs=nano.specs
#    -Wl,-Map=${PROJECT_NAME}.map
#    -u _printf_float
#)


#add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
#    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
#    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
#    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
#)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}                      # -mcpu=cortex-m0plus, -mthumb, -mfloat-abi=soft
    -T "${LINKER_SCRIPT}"             # Linker script megadása
    -Wl,-Map=${PROJECT_NAME}.map      # Map fájl generálása
    -Wl,--gc-sections                 # Nem használt szekciók törlése
    -static                           # Statikus linkelés
    -flto                             # Link-time optimization (kisebb kódméret)
    -Wl,--print-memory-usage          # EZ generálja a kért táblázatot a RAM/FLASH használatról
    --specs=nano.specs                # Newlib-nano használata
    --specs=nosys.specs               # Bare-metal rendszerhívásokhoz
    -Wl,--start-group -lc -lm -Wl,--end-group # Standard C és Math könyvtárak
    -u _printf_float                  # Float támogatás printf-hez
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
)

set_target_properties(${PROJECT_NAME}.elf PROPERTIES SUFFIX "")
