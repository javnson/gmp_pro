cmake_minimum_required(VERSION 3.22)
set(CMAKE_PROJECT_NAME stm32f411re_nucleo)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(${CMAKE_PROJECT_NAME} C ASM)

set(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")
set(HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver")
set(CMSIS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
file(GLOB GMP_GENERATED_SOURCES CONFIGURE_DEPENDS "${SHARED_DIR}/gmp_src_mgr/gmp_src/*.c")

set(HAL_SOURCES
    ${HAL_DIR}/Src/stm32f4xx_hal.c
    ${HAL_DIR}/Src/stm32f4xx_hal_adc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_adc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_exti.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash_ramfunc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2c.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2c_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_tim.c
    ${HAL_DIR}/Src/stm32f4xx_hal_tim_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_uart.c
)

add_executable(${CMAKE_PROJECT_NAME}
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/system_stm32f4xx.c
    cmake/startup_stm32f411xe.s
    ${HAL_SOURCES}
    ${SHARED_DIR}/user/user_main.c
    ${SHARED_DIR}/user/user_dl.c
    ${SHARED_DIR}/user/ctl_main.c
    ${SHARED_DIR}/xplt/xplt.peripheral.c
    ${GMP_GENERATED_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32F4xx/Include
    ${CMSIS_DIR}/Include
    ${SHARED_DIR}/user
    ${SHARED_DIR}/xplt
    ${SHARED_DIR}/gmp_src_mgr/gmp_inc
    ${SHARED_DIR}/gmp_src_mgr/gmp_inc/csp/stm32
    sdpe_mgr
    ../../../..
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_HAL_DRIVER STM32F411xE)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -ffunction-sections -fdata-sections)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F411RETX_FLASH.ld
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE m)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
)
