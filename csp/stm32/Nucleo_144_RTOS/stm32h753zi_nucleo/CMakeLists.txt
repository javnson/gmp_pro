cmake_minimum_required(VERSION 3.22)
set(CMAKE_PROJECT_NAME stm32h753zi_nucleo_rtos)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(${CMAKE_PROJECT_NAME} C ASM)

set(RTOS_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")
set(BAREMETAL_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Nucleo_144/src")
set(HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver")
set(CMSIS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
set(FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source")
file(GLOB GMP_GENERATED_SOURCES CONFIGURE_DEPENDS
    "${RTOS_SHARED_DIR}/gmp_src_mgr/gmp_src/*.c")

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

set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM7/r0p1/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

add_executable(${CMAKE_PROJECT_NAME}
    Core/Src/main.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/system_stm32h7xx.c
    cmake/startup_stm32h753xx.s
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${BAREMETAL_SHARED_DIR}/user/user_dl.c
    ${BAREMETAL_SHARED_DIR}/user/ctl_main.c
    ${RTOS_SHARED_DIR}/rtos/gmp_stm32_freertos.c
    ${RTOS_SHARED_DIR}/user/user_main.c
    ${RTOS_SHARED_DIR}/user/rtos_app.c
    ${RTOS_SHARED_DIR}/xplt/xplt.peripheral.c
    ${GMP_GENERATED_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    config
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
    ${CMSIS_DIR}/Include
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM7/r0p1
    ${RTOS_SHARED_DIR}/rtos
    ${RTOS_SHARED_DIR}/xplt
    ${BAREMETAL_SHARED_DIR}/user
    ${BAREMETAL_SHARED_DIR}/xplt
    ${RTOS_SHARED_DIR}/user
    ${RTOS_SHARED_DIR}/gmp_src_mgr/gmp_inc
    ${RTOS_SHARED_DIR}/gmp_src_mgr/gmp_inc/csp/stm32
    sdpe_mgr
    ../../../..
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32H753xx
    SPECIFY_GMP_OS_BACKEND=GMP_OS_BACKEND_FREERTOS
    GMP_DL_RX_FIFO_SIZE=512
    GMP_NUCLEO_ENABLE_ETHERNET_DL=0
)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall -Wextra -ffunction-sections -fdata-sections)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32H753ZITX_FLASH.ld
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE m)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
)
