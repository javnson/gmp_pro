cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(PMSM_Sensored_MC_simulink C CXX)
set(EXECUTABLE_NAME motor_control_simulink)

# 设置C和C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CXX_STANDARD_REQUIRED ON)

# 检测操作系统平台
if(WIN32)
    set(OS_WINDOWS TRUE)
    message(STATUS "Building for Windows")
elseif(UNIX AND NOT APPLE)
    set(OS_LINUX TRUE)
    message(STATUS "Building for Linux")
else()
    message(FATAL_ERROR "Unsupported operating system")
endif()

# 定义源代码目录
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(GMP_SRC_DIR ${SOURCE_DIR}/gmp_src)
set(IMPLEMENT_DIR ${SOURCE_DIR}/../../implement)

# 确保路径在Linux和Windows上都正常工作
string(REPLACE "\\" "/" SOURCE_DIR_CLEAN ${SOURCE_DIR})
string(REPLACE "\\" "/" GMP_SRC_DIR_CLEAN ${GMP_SRC_DIR})
string(REPLACE "\\" "/" IMPLEMENT_DIR_CLEAN ${IMPLEMENT_DIR})

# 从环境变量读取GMP_PRO_LOCATION
if(DEFINED ENV{GMP_PRO_LOCATION})
    set(GMP_PRO_LOCATION $ENV{GMP_PRO_LOCATION})
    message(STATUS "GMP_PRO_LOCATION set from environment: ${GMP_PRO_LOCATION}")
else()
    # 如果环境变量未定义，使用默认路径
    set(GMP_PRO_LOCATION ${SOURCE_DIR}/../../../../..)
    message(STATUS "GMP_PRO_LOCATION set from default: ${GMP_PRO_LOCATION}")
endif()
string(REPLACE "\\" "/" GMP_PRO_LOCATION_CLEAN ${GMP_PRO_LOCATION})

# 手动列出所有源文件
set(C_SOURCE_FILES
    ${IMPLEMENT_DIR}/common/ctl_main.c
    ${IMPLEMENT_DIR}/common/user_main.c
    ${IMPLEMENT_DIR}/simulate/xplt.peripheral.c
    ${GMP_SRC_DIR}/ctl_component_advance.c
    ${GMP_SRC_DIR}/ctl_component_basic.c
    ${GMP_SRC_DIR}/ctl_component_continuous.c
    ${GMP_SRC_DIR}/ctl_component_discrete.c
    ${GMP_SRC_DIR}/ctl_component_interface.c
    ${GMP_SRC_DIR}/ctl_component_protection.c
    ${GMP_SRC_DIR}/ctl_mc_acm_sensored_ctrl.c
    ${GMP_SRC_DIR}/ctl_mc_basic.c
    ${GMP_SRC_DIR}/ctl_mc_consultant.c
    ${GMP_SRC_DIR}/ctl_mc_current_loop.c
    ${GMP_SRC_DIR}/ctl_mc_motion.c
    ${GMP_SRC_DIR}/ctl_mc_observer.c
    ${GMP_SRC_DIR}/ctl_mc_pmsm_ctrl.c
    ${GMP_SRC_DIR}/ctl_mc_pmsm_ctrl_hfi.c
    ${GMP_SRC_DIR}/ctl_mc_pmsm_ctrl_mtpa.c
    ${GMP_SRC_DIR}/ctl_mc_pmsm_ctrl_smo.c
    ${GMP_SRC_DIR}/ctl_mc_pmsm_offline_est.c
    ${GMP_SRC_DIR}/ctl_nano.c
    ${GMP_SRC_DIR}/gmp_dev_util.c
    ${GMP_SRC_DIR}/gmp_ds_list.c
    ${GMP_SRC_DIR}/gmp_mm_block_memory.c
    ${GMP_SRC_DIR}/gmp_std_error_code.c
    ${GMP_SRC_DIR}/gmp_std_port.c
)

# 条件性地添加平台特定文件
set(CPP_SOURCE_FILES
    ${SOURCE_DIR}/motor_control_simulink.cpp
)

if(OS_WINDOWS)
    # 使用简化版的Windows Simulink主文件，避免Boost和nlohmann/json依赖
    list(APPEND CPP_SOURCE_FILES ${GMP_SRC_DIR}/windows_simulink_main_simple.cpp)
elseif(OS_LINUX)
    # 在Linux上，我们可能需要一个替代的main实现
    if(EXISTS ${GMP_SRC_DIR}/linux_simulink_main.cpp)
        list(APPEND CPP_SOURCE_FILES ${GMP_SRC_DIR}/linux_simulink_main.cpp)
    else()
        # 如果没有Linux特定版本，则仍然使用windows版本并通过条件编译处理
        list(APPEND CPP_SOURCE_FILES ${GMP_SRC_DIR}/windows_simulink_main.cpp)
        add_definitions(-DGMP_LINUX_PORT)
    endif()
endif()

# 合并所有源文件
set(SOURCE_FILES
    ${C_SOURCE_FILES}
    ${CPP_SOURCE_FILES}
)

# 设置包含目录
include_directories(
    ${IMPLEMENT_DIR}/common
    ${IMPLEMENT_DIR}/simulate
    ${SOURCE_DIR}
    ${GMP_SRC_DIR}
    ${GMP_PRO_LOCATION}  # 确保tools/gmp_sil路径可以被正确包含
    # 第三方库路径
    ${GMP_PRO_LOCATION}/ctl/suite/motor_control/pmsm/implement/user_common
    ${GMP_PRO_LOCATION}/ctl/suite/motor_control/pmsm/implement/user_simulink
    ${GMP_PRO_LOCATION}/third_party/asio/include
    ${GMP_PRO_LOCATION}/third_party/eigen/Eigen
    ${GMP_PRO_LOCATION}/csp/windows_simulink
    ${GMP_PRO_LOCATION}/third_party/fmt/include/fmt
)

# 添加平台特定的包含目录
if(OS_WINDOWS)
    include_directories(
        ${GMP_PRO_LOCATION}/csp/windows_simulink
    )
elseif(OS_LINUX)
    # Linux特定的包含目录
    include_directories(
        ${GMP_PRO_LOCATION}/csp/linux_simulink  # 假设有这个目录，实际可能需要调整
    )
endif()

# 添加预处理器定义
add_definitions(
    -D_CONSOLE
)

# 平台特定的编译选项
if(OS_WINDOWS)
    add_definitions(-DWIN32)
    add_compile_options(/W3)
    # 设置字符集为Unicode
    add_compile_options(/utf-8)
elseif(OS_LINUX)
    # Linux特定的编译选项
    add_compile_options(-Wall -Wextra -Wpedantic)
    # 确保在Linux上编译时不会出现未定义引用
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined")
endif()

# 创建可执行文件
add_executable(${EXECUTABLE_NAME} ${SOURCE_FILES})

# 使用生成器表达式设置配置相关选项
target_compile_definitions(${EXECUTABLE_NAME} PRIVATE
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
)

target_compile_options(${EXECUTABLE_NAME} PRIVATE
    # 调试配置
    $<$<CONFIG:Debug>:
        $<$<BOOL:${OS_WINDOWS}>:/Od /Zi /RTC1 /W4>
        $<$<BOOL:${OS_LINUX}>:-g -O0 -ggdb3>
    >
    # 发布配置
    $<$<CONFIG:Release>:
        $<$<BOOL:${OS_WINDOWS}>:/O2 /GL /Gy>
        $<$<BOOL:${OS_LINUX}>:-O2>
    >
)

# 添加平台特定的库链接
if(OS_WINDOWS)
    # Windows特定库
    target_link_libraries(${EXECUTABLE_NAME})
    
    # 添加Visual Studio属性表
    if(MSVC)
        # 定义属性表路径
        set(CSP_PROPERTY_SHEET "${GMP_PRO_LOCATION}/csp/windows_simulink/GMPCorePropertySheet.props")
        set(TOOLS_PROPERTY_SHEET "${GMP_PRO_LOCATION}/tools/facilities_generator/GMPSrcGenPropertySheet.props")
        
        # 检查并添加属性表
        set_property(TARGET ${EXECUTABLE_NAME} PROPERTY VS_USER_PROPS "")
        
        if(EXISTS ${CSP_PROPERTY_SHEET})
            message(STATUS "Adding CSP property sheet: ${CSP_PROPERTY_SHEET}")
            set_property(TARGET ${EXECUTABLE_NAME} APPEND PROPERTY VS_USER_PROPS "${CSP_PROPERTY_SHEET}")
        else()
            message(WARNING "CSP property sheet not found: ${CSP_PROPERTY_SHEET}")
        endif()
        
        if(EXISTS ${TOOLS_PROPERTY_SHEET})
            message(STATUS "Adding tools property sheet: ${TOOLS_PROPERTY_SHEET}")
            set_property(TARGET ${EXECUTABLE_NAME} APPEND PROPERTY VS_USER_PROPS "${TOOLS_PROPERTY_SHEET}")
        else()
            message(WARNING "Tools property sheet not found: ${TOOLS_PROPERTY_SHEET}")
        endif()
        
        # 设置项目配置
        set_target_properties(${EXECUTABLE_NAME} PROPERTIES
            VS_GLOBAL_ProjectName "PMSM_Sensored_MC_simulink"
            VS_GLOBAL_Keyword "Win32Proj"
            VS_GLOBAL_RootNamespace "motorcontrolsimulink"
            VS_GLOBAL_WindowsTargetPlatformVersion "10.0"
        )
    endif()
    
    # 添加预构建事件（仅Debug x64配置）
    if(MSVC AND "${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
        add_custom_command(
            TARGET ${EXECUTABLE_NAME} 
            PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_SOURCE_DIR} gmp_fac_generate_src.bat
            COMMENT "Running pre-build script for facilities generation"
            VERBATIM
        )
    endif()
elseif(OS_LINUX)
    # Linux特定库
    target_link_libraries(${EXECUTABLE_NAME} 
        pthread
        stdc++fs  # 确保C++文件系统库可用
        dl        # 动态加载库
    )
    # 确保二进制文件可执行权限
    set_target_properties(${EXECUTABLE_NAME} PROPERTIES
        LINK_FLAGS "-Xlinker -rpath=$ORIGIN"
    )
endif()

# 设置安装目标，便于在Linux上部署
install(TARGETS ${EXECUTABLE_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# 添加调试配置支持
if(OS_LINUX AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Adding GDB debugging support")
    # 可以在这里添加自定义的调试目标或命令
endif()

# 添加对Win32平台的支持
if(OS_WINDOWS)
    # 确保可以为Win32和x64平台构建
    if(NOT DEFINED CMAKE_GENERATOR_PLATFORM)
        message(STATUS "No platform specified, default to current system architecture")
    else()
        message(STATUS "Building for platform: ${CMAKE_GENERATOR_PLATFORM}")
    endif()
endif()
