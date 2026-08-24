# GMP-owned vcpkg selection for CMake/Visual Studio folder projects.
#
# Private installations consume the immutable package set prepared below
# GMP_PRO_LOCATION/bin.  System installations retain normal vcpkg manifest
# restore semantics and use the vcpkg visible in the user's environment.

include_guard(GLOBAL)

if("$ENV{GMP_PRO_LOCATION}" STREQUAL "")
    message(FATAL_ERROR
        "GMP_PRO_LOCATION is not defined. Run a GMP installer, then restart "
        "the CMake/Visual Studio process.")
endif()

file(TO_CMAKE_PATH "$ENV{GMP_PRO_LOCATION}" _GMP_VCPKG_ROOT_DIR)
set(_GMP_PRIVATE_MARKER
    "${_GMP_VCPKG_ROOT_DIR}/bin/gmp_virtual_env_installed.flag")
set(_GMP_PRIVATE_VCPKG
    "${_GMP_VCPKG_ROOT_DIR}/bin/vcpkg")

option(GMP_VCPKG_FORCE_SYSTEM
    "Ignore a private marker and test/use the system vcpkg path" OFF)

if(EXISTS "${_GMP_PRIVATE_MARKER}" AND NOT GMP_VCPKG_FORCE_SYSTEM)
    set(_GMP_VCPKG_TOOLCHAIN
        "${_GMP_PRIVATE_VCPKG}/scripts/buildsystems/vcpkg.cmake")
    set(_GMP_PRIVATE_INSTALLED
        "${_GMP_VCPKG_ROOT_DIR}/bin/vcpkg_installed/x64-windows")
    if(NOT EXISTS "${_GMP_VCPKG_TOOLCHAIN}")
        message(FATAL_ERROR
            "The GMP private environment marker exists, but its vcpkg "
            "toolchain is missing. Repair the GMP private environment.")
    endif()
    if(NOT IS_DIRECTORY "${_GMP_PRIVATE_INSTALLED}/x64-windows/share")
        message(FATAL_ERROR
            "The GMP private vcpkg package tree is incomplete. Run "
            "tools/gmp_installer/utilities/repair_gmp_vcpkg.bat.")
    endif()

    # The private installer has already restored the aggregate dependency set.
    # Prevent a project manifest from creating another installed tree.
    set(VCPKG_INSTALLED_DIR "${_GMP_PRIVATE_INSTALLED}" CACHE PATH
        "GMP private shared vcpkg installed root" FORCE)
    set(VCPKG_MANIFEST_MODE OFF CACHE BOOL
        "Use the package set prepared by the GMP private installer" FORCE)
    set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING
        "GMP private vcpkg target triplet" FORCE)
    message(STATUS
        "GMP vcpkg mode: private shared packages (${_GMP_PRIVATE_INSTALLED})")
else()
    # System mode intentionally leaves manifest mode enabled. Prefer the
    # registered VCPKG_ROOT, then resolve the executable from PATH.
    file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}" _GMP_SYSTEM_VCPKG)
    if(NOT EXISTS "${_GMP_SYSTEM_VCPKG}/scripts/buildsystems/vcpkg.cmake")
        find_program(_GMP_VCPKG_EXECUTABLE NAMES vcpkg vcpkg.exe)
        if(NOT _GMP_VCPKG_EXECUTABLE)
            message(FATAL_ERROR
                "System GMP mode requires vcpkg on PATH or VCPKG_ROOT.")
        endif()
        get_filename_component(
            _GMP_SYSTEM_VCPKG "${_GMP_VCPKG_EXECUTABLE}" DIRECTORY)
    endif()
    set(_GMP_VCPKG_TOOLCHAIN
        "${_GMP_SYSTEM_VCPKG}/scripts/buildsystems/vcpkg.cmake")
    if(NOT EXISTS "${_GMP_VCPKG_TOOLCHAIN}")
        message(FATAL_ERROR
            "Cannot locate the system vcpkg CMake toolchain under "
            "${_GMP_SYSTEM_VCPKG}.")
    endif()
    message(STATUS
        "GMP vcpkg mode: system manifest restore (${_GMP_SYSTEM_VCPKG})")
endif()

include("${_GMP_VCPKG_TOOLCHAIN}")
