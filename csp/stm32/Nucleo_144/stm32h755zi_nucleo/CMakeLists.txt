cmake_minimum_required(VERSION 3.22)
set(CMAKE_PROJECT_NAME stm32h755zi_nucleo)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(${CMAKE_PROJECT_NAME} C ASM)

set(GMP_DATALINK_TRANSPORT "TCP" CACHE STRING "CM7 GMP Data Link Ethernet transport (TCP or UDP)")
set_property(CACHE GMP_DATALINK_TRANSPORT PROPERTY STRINGS TCP UDP)
string(TOUPPER "${GMP_DATALINK_TRANSPORT}" GMP_DATALINK_TRANSPORT_NORMALIZED)
if(GMP_DATALINK_TRANSPORT_NORMALIZED STREQUAL "TCP")
    set(GMP_DATALINK_TRANSPORT_VALUE 1)
elseif(GMP_DATALINK_TRANSPORT_NORMALIZED STREQUAL "UDP")
    set(GMP_DATALINK_TRANSPORT_VALUE 2)
else()
    message(FATAL_ERROR "GMP_DATALINK_TRANSPORT must be TCP or UDP")
endif()

set(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")
set(HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver")
set(CMSIS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
set(LWIP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/LwIP")
set(CM7_LWIP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CM7/LWIP")
file(GLOB GMP_GENERATED_SOURCES CONFIGURE_DEPENDS "${SHARED_DIR}/gmp_src_mgr/gmp_src/*.c")
file(GLOB LWIP_CORE_SOURCES CONFIGURE_DEPENDS "${LWIP_DIR}/src/core/*.c")
file(GLOB LWIP_IPV4_SOURCES CONFIGURE_DEPENDS "${LWIP_DIR}/src/core/ipv4/*.c")

set(HAL_SOURCES
    ${HAL_DIR}/Src/stm32h7xx_hal.c
    ${HAL_DIR}/Src/stm32h7xx_hal_adc.c
    ${HAL_DIR}/Src/stm32h7xx_hal_adc_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_dma.c
    ${HAL_DIR}/Src/stm32h7xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_eth.c
    ${HAL_DIR}/Src/stm32h7xx_hal_eth_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_exti.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32h7xx_hal_hsem.c
    ${HAL_DIR}/Src/stm32h7xx_hal_i2c.c
    ${HAL_DIR}/Src/stm32h7xx_hal_i2c_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_tim.c
    ${HAL_DIR}/Src/stm32h7xx_hal_tim_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart_ex.c
)

set(GMP_USER_SOURCES
    ${SHARED_DIR}/user/user_main.c
    ${SHARED_DIR}/user/user_dl.c
    ${SHARED_DIR}/user/ctl_main.c
    ${SHARED_DIR}/xplt/xplt.peripheral.c
    ${GMP_GENERATED_SOURCES}
)

function(configure_core target core_dir cpu_flags linker_script)
    target_include_directories(${target} PRIVATE
        ${core_dir}/Core/Inc
        ${HAL_DIR}/Inc
        ${HAL_DIR}/Inc/Legacy
        ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
        ${CMSIS_DIR}/Include
        ${SHARED_DIR}/user
        ${SHARED_DIR}/xplt
        ${SHARED_DIR}/gmp_src_mgr/gmp_inc
        ${SHARED_DIR}/gmp_src_mgr/gmp_inc/csp/stm32
        sdpe_mgr
        ../../../..
    )
    target_compile_definitions(${target} PRIVATE
        USE_HAL_DRIVER STM32H755xx
        GMP_DL_RX_FIFO_SIZE=512
        GMP_NUCLEO_DUAL_CORE=1
    )
    target_compile_options(${target} PRIVATE
        ${cpu_flags} -Wall -Wextra -ffunction-sections -fdata-sections
    )
    target_link_options(${target} PRIVATE
        ${cpu_flags} -T${linker_script}
        --specs=nano.specs --specs=nosys.specs
        -Wl,-Map=${target}.map -Wl,--gc-sections -Wl,--print-memory-usage
    )
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${linker_script}")
    target_link_libraries(${target} PRIVATE m)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${CMAKE_CURRENT_BINARY_DIR}/${target}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
    )
endfunction()

set(CM7_TARGET stm32h755zi_nucleo_cm7)
set(CM7_CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
add_executable(${CM7_TARGET}
    CM7/Core/Src/main.c CM7/Core/Src/stm32h7xx_it.c
    CM7/Core/Src/stm32h7xx_hal_msp.c CM7/Core/Src/system_stm32h7xx.c
    cmake/startup_stm32h755xx_cm7.s ${HAL_SOURCES}
    ${CM7_LWIP_DIR}/App/lwip.c ${CM7_LWIP_DIR}/Target/ethernetif.c
    Drivers/BSP/Components/lan8742/lan8742.c
    ${LWIP_CORE_SOURCES} ${LWIP_IPV4_SOURCES}
    ${LWIP_DIR}/src/netif/ethernet.c
    ${SHARED_DIR}/xplt/xplt.ethernet.c ${GMP_USER_SOURCES}
)
configure_core(${CM7_TARGET} "${CMAKE_CURRENT_SOURCE_DIR}/CM7" "${CM7_CPU_FLAGS}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32H755ZITX_CM7_FLASH.ld")
target_include_directories(${CM7_TARGET} PRIVATE
    Drivers/BSP/Components/lan8742 ${CM7_LWIP_DIR}/App ${CM7_LWIP_DIR}/Target
    ${LWIP_DIR}/src/include ${LWIP_DIR}/system
)
target_compile_definitions(${CM7_TARGET} PRIVATE
    CORE_CM7 HAL_UART_MODULE_ENABLED
    GMP_NUCLEO_ENABLE_UART_DL=0 GMP_NUCLEO_ENABLE_ETHERNET_DL=1
    GMP_NUCLEO_ENABLE_CONTROL=1 GMP_NUCLEO_ENABLE_STATUS_LED=1
    GMP_NUCLEO_STATUS_LED_INDEX=1
    GMP_NUCLEO_ETH_DL_TRANSPORT=${GMP_DATALINK_TRANSPORT_VALUE}
)

set(CM4_TARGET stm32h755zi_nucleo_cm4)
set(CM4_CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
add_executable(${CM4_TARGET}
    CM4/Core/Src/main.c CM4/Core/Src/stm32h7xx_it.c
    CM4/Core/Src/stm32h7xx_hal_msp.c CM4/Core/Src/system_stm32h7xx.c
    cmake/startup_stm32h755xx_cm4.s ${HAL_SOURCES} ${GMP_USER_SOURCES}
)
configure_core(${CM4_TARGET} "${CMAKE_CURRENT_SOURCE_DIR}/CM4" "${CM4_CPU_FLAGS}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32H755ZITX_CM4_FLASH.ld")
target_compile_definitions(${CM4_TARGET} PRIVATE
    CORE_CM4 GMP_NUCLEO_ENABLE_UART_DL=1 GMP_NUCLEO_ENABLE_ETHERNET_DL=0
    GMP_NUCLEO_ENABLE_CONTROL=0 GMP_NUCLEO_ENABLE_STATUS_LED=1
    GMP_NUCLEO_STATUS_LED_INDEX=2
)
