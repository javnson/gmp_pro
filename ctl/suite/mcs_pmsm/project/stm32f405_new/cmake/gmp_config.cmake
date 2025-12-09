# GMP library configuration

# This file should be copyied to <stm32_proj>/cmake/gmp_config.cmake
# The following configs should added to <stm32_proj>/CMakeLists.txt
# include(cmake/gmp_config.cmake)
# after add all cube MX generated sources

# 1. 在编译前调用gmp_fac_generate_src.bat脚本
add_custom_target(generate_gmp_src
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/../gmp_fac_generate_src.bat
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMENT "Generating/updating GMP source files..."
)

# 2. 将gmp_src文件夹中的所有文件添加到源文件中
file(GLOB GMP_SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/../gmp_src/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${GMP_SRC_FILES})

# 3. 将common文件夹中的源文件添加到编译文件中
set(COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../implement/common")
file(GLOB COMMON_SRC_FILES
    ${COMMON_DIR}/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${COMMON_SRC_FILES})

# 4. 将stm32f405文件夹中的源文件添加到编译文件中

# 获取当前CMake文件所在目录的父目录名称（即project目录名称，如stm32f405）
get_filename_component(CURRENT_PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
get_filename_component(PROJECT_NAME ${CURRENT_PROJECT_DIR} NAME)

# 动态设置STM32_SRC_DIR路径
set(STM32_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../implement/${PROJECT_NAME}")
file(GLOB STM32_SRC_FILES
    ${STM32_SRC_DIR}/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${STM32_SRC_FILES})

# 5. 添加include路径
# 获取环境变量GMP_PRO_LOCATION
if(DEFINED ENV{GMP_PRO_LOCATION})
    set(GMP_PRO_DIR $ENV{GMP_PRO_LOCATION})
else()
    # 如果环境变量不存在，则报错
    message(FATAL_ERROR "Environment variable GMP_PRO_LOCATION is not set. Please install GMP first.")
endif()

# 添加include路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${COMMON_DIR}
    ${STM32_SRC_DIR}
    ${GMP_PRO_DIR}
    ${GMP_PRO_DIR}/csp/stm32  # 添加STM32的CSP配置目录
)

# 将generate_gmp_src目标设为构建依赖
add_dependencies(${CMAKE_PROJECT_NAME} generate_gmp_src)