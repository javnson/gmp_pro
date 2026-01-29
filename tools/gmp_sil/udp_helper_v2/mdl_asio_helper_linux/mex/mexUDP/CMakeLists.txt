cmake_minimum_required(VERSION 3.15)
project(GMP_SIL_Project LANGUAGES CXX)

set(TARGET_NAME GMP_SIL_Core)
set(SOURCE_FILE mdl_asio_helper.cpp)

# 1. 查找依赖项
find_package(Matlab REQUIRED COMPONENTS MX_LIBRARY)
find_package(asio CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)

# 2. 清洗并构造 MATLAB 核心路径
set(RAW_MATLAB_ROOT "$ENV{MATLAB_ROOT}")
string(STRIP "${RAW_MATLAB_ROOT}" STRIPPED_MATLAB_ROOT)

# 根据你的 find 结果增加 simulink/include 路径
set(MATLAB_INC_PATHS
    "${STRIPPED_MATLAB_ROOT}/extern/include"
    "${STRIPPED_MATLAB_ROOT}/simulink/include"   # 修复 fixedpoint.h 找不到的问题
)

# 3. 获取自定义库路径
set(GMP_PRO_LIB_DIR "$ENV{GMP_PRO_LOCATION}")
if(NOT GMP_PRO_LIB_DIR)
    message(FATAL_ERROR "Environment variable GMP_PRO_LOCATION is not set!")
endif()

# 4. 定义目标
add_library(${TARGET_NAME} SHARED ${SOURCE_FILE})

# 5. 配置包含目录
target_include_directories(${TARGET_NAME} PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${GMP_PRO_LIB_DIR}"
    ${MATLAB_INC_PATHS}        # 注入修复后的路径列表
    ${Matlab_INCLUDE_DIRS}
)

# 6. 配置链接库
target_link_libraries(${TARGET_NAME} PRIVATE 
    ${Matlab_LIBRARIES}
    asio::asio 
    nlohmann_json::nlohmann_json 
    fmt::fmt
    Threads::Threads
)

# 7. 宏定义 (S-Function 必备)
target_compile_definitions(${TARGET_NAME} PRIVATE 
    MATLAB_MEX_FILE 
    ASIO_STANDALONE
)

# 8. 平台属性与后缀名
if(UNIX)
    set_target_properties(${TARGET_NAME} PROPERTIES PREFIX "" SUFFIX ".mexa64")
    target_compile_options(${TARGET_NAME} PRIVATE -fPIC)
else()
    set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".mexw64")
endif()