@echo off

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined.
    echo         Run install_gmp.bat, install_gmp_virtual_env.bat, or register_gmp_root.bat first.
    exit /b 1
)
if not exist "%GMP_PRO_LOCATION%\tools\gmp_installer\environment_manifest.json" (
    echo [ERROR] GMP_PRO_LOCATION does not point to a valid GMP Pro repository:
    echo         %GMP_PRO_LOCATION%
    exit /b 1
)
for %%I in ("%GMP_PRO_LOCATION%") do set "GMP_PRO_LOCATION=%%~fI"
set "GMP_BIN=%GMP_PRO_LOCATION%\bin"

if not exist "%GMP_BIN%\python\python.exe" (
    echo [ERROR] GMP private environment is not installed at "%GMP_BIN%".
    echo         Run install_gmp_virtual_env.bat or copy bin and run deploy_gmp_env.bat first.
    exit /b 1
)

if not exist "%GMP_BIN%\cache\vcpkg-downloads" (
    mkdir "%GMP_BIN%\cache\vcpkg-downloads"
    if errorlevel 1 (
        echo [ERROR] Cannot create the vcpkg downloads cache.
        exit /b 1
    )
)
if not exist "%GMP_BIN%\cache\vcpkg-archives" (
    mkdir "%GMP_BIN%\cache\vcpkg-archives"
    if errorlevel 1 (
        echo [ERROR] Cannot create the vcpkg binary cache.
        exit /b 1
    )
)

if exist "%GMP_BIN%\gmp_proxy_env.bat" (
    call "%GMP_BIN%\gmp_proxy_env.bat"
    if errorlevel 1 (
        echo [ERROR] Cannot load the GMP private proxy configuration.
        exit /b 1
    )
)

if not "%GMP_ENV_ACTIVE%"=="%GMP_PRO_LOCATION%" (
    set "GMP_ENV_ACTIVE=%GMP_PRO_LOCATION%"
    set "GMP_ENV_MODE=virtual"
    set "PYTHONHOME=%GMP_BIN%\python"
    set "PYTHONNOUSERSITE=1"
    set "PYTHONUTF8=1"
    set "PIP_DISABLE_PIP_VERSION_CHECK=1"
    set "VCPKG_ROOT=%GMP_BIN%\vcpkg"
    set "VCPKG_DOWNLOADS=%GMP_BIN%\cache\vcpkg-downloads"
    set "VCPKG_DEFAULT_BINARY_CACHE=%GMP_BIN%\cache\vcpkg-archives"
    set "VCPKG_INSTALLED_DIR=%GMP_BIN%\vcpkg_installed\x64-windows"
    set "VCPKG_DEFAULT_TRIPLET=x64-windows"
    set "VCPKG_FEATURE_FLAGS=manifests,binarycaching"
    set "CMAKE_TOOLCHAIN_FILE=%GMP_BIN%\vcpkg\scripts\buildsystems\vcpkg.cmake"
    set "PATH=%GMP_BIN%\python;%GMP_BIN%\python\Scripts;%GMP_BIN%\apps\git\cmd;%GMP_BIN%\apps\git\mingw64\bin;%GMP_BIN%\apps\cmake\bin;%GMP_BIN%\apps\ninja;%GMP_BIN%\apps\doxygen;%GMP_BIN%\apps\graphviz\bin;%GMP_BIN%\vcpkg;%PATH%"
)
set "GMP_ENV_MODE=virtual"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" if not defined VSCMD_VER for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do if exist "%%I\Common7\Tools\VsDevCmd.bat" call "%%I\Common7\Tools\VsDevCmd.bat" -no_logo -arch=x64 -host_arch=x64

rem VsDevCmd may select Visual Studio's bundled vcpkg. Reassert GMP ownership.
set "VCPKG_ROOT=%GMP_BIN%\vcpkg"
set "VCPKG_DOWNLOADS=%GMP_BIN%\cache\vcpkg-downloads"
set "VCPKG_DEFAULT_BINARY_CACHE=%GMP_BIN%\cache\vcpkg-archives"
set "VCPKG_INSTALLED_DIR=%GMP_BIN%\vcpkg_installed\x64-windows"
set "VCPKG_DEFAULT_TRIPLET=x64-windows"
set "VCPKG_FEATURE_FLAGS=manifests,binarycaching"
set "CMAKE_TOOLCHAIN_FILE=%GMP_BIN%\vcpkg\scripts\buildsystems\vcpkg.cmake"

cd /d "%GMP_PRO_LOCATION%"
echo.
echo [GMP] Private development environment activated.
echo [GMP] Root   : %GMP_PRO_LOCATION%
echo [GMP] Python : %GMP_BIN%\python\python.exe
echo [GMP] vcpkg  : %VCPKG_ROOT%\vcpkg.exe
if defined GMP_PROXY_MODE echo [GMP] Network: %GMP_PROXY_MODE%
if not defined VSCMD_VER echo [WARN] Visual Studio C++ developer tools were not detected.
echo.
exit /b 0
