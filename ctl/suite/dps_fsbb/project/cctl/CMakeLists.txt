cmake_minimum_required(VERSION 3.21)
project(dps_fsbb_cctl LANGUAGES C CXX)

set(FSBB_BUILD_LEVEL "3" CACHE STRING "FSBB control level (1, 2, or 3)")
if(NOT FSBB_BUILD_LEVEL MATCHES "^[123]$")
    message(FATAL_ERROR "FSBB_BUILD_LEVEL must be 1, 2, or 3.")
endif()
find_package(Threads REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

set(GMP_CMAKE_TARGET dps_fsbb_cctl)
set(FSBB_PLANT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(GMP_CMAKE_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/gmp_src_mgr/gmp_config.cmake")
if(NOT EXISTS "${GMP_CMAKE_CONFIG}" OR
   NOT EXISTS "${FSBB_PLANT_DIR}/fsbb_closed_loop.archive" OR
   NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdpe_mgr/ctrl_settings.h")
    message(FATAL_ERROR "Run build_test.bat to generate SDPE, sources, and the FSBB plant.")
endif()

add_executable(dps_fsbb_cctl fsbb_simulation.cpp ../../src/ctl_main.c ../../src/user_main.c)
include("${GMP_CMAKE_CONFIG}")
target_include_directories(dps_fsbb_cctl PRIVATE
    xplt sdpe_mgr ../../src "${FSBB_PLANT_DIR}")
target_compile_features(dps_fsbb_cctl PRIVATE c_std_11 cxx_std_17)
target_compile_definitions(dps_fsbb_cctl PRIVATE
    CCTL_FSBB_BUILD_LEVEL=${FSBB_BUILD_LEVEL}
    FSBB_BUILD_CONFIGURATION="$<CONFIG>"
    FSBB_OPTIMIZED_BUILD=$<IF:$<CONFIG:Debug>,0,1>)
target_link_libraries(dps_fsbb_cctl PRIVATE
    Threads::Threads Eigen3::Eigen nlohmann_json::nlohmann_json)
if(MSVC)
    target_compile_options(dps_fsbb_cctl PRIVATE /utf-8 /bigobj /wd4828)
endif()
add_custom_command(TARGET dps_fsbb_cctl POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${FSBB_PLANT_DIR}/fsbb_closed_loop.archive" "$<TARGET_FILE_DIR:dps_fsbb_cctl>"
    VERBATIM)
set_target_properties(dps_fsbb_cctl PROPERTIES
    LINK_DEPENDS "${FSBB_PLANT_DIR}/fsbb_closed_loop.archive"
    VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:dps_fsbb_cctl>")

enable_testing()
add_test(NAME fsbb_cctl_level_${FSBB_BUILD_LEVEL}
    COMMAND dps_fsbb_cctl --headless --no-pause --normal-priority
            --output fsbb_level_${FSBB_BUILD_LEVEL}.csv)
set_tests_properties(fsbb_cctl_level_${FSBB_BUILD_LEVEL} PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:dps_fsbb_cctl>")
