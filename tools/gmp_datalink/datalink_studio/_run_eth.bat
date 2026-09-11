@echo off
setlocal EnableExtensions

if "%~2"=="" exit /b 2
set "GMP_DATALINK_TRANSPORT=%~1"
set "GMP_DATALINK_TARGET_UNIT_BYTES=%~2"

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    pause
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

python -c "import PyQt5, serial, pyqtgraph, numpy"
if errorlevel 1 (
    echo [ERROR] GMP Data Link Studio dependencies are incomplete.
    echo         Re-run install_gmp_virtual_env.bat or
    echo         tools\gmp_installer\utilities\install_gmp.bat.
    pause
    exit /b 1
)

if not defined GMP_DATALINK_ETH_HOST set "GMP_DATALINK_ETH_HOST=192.168.137.2"
if not defined GMP_DATALINK_ETH_PORT (
    if /I "%GMP_DATALINK_TRANSPORT%"=="tcp" (
        set "GMP_DATALINK_ETH_PORT=50001"
    ) else (
        set "GMP_DATALINK_ETH_PORT=50002"
    )
)

if "%GMP_DATALINK_TARGET_UNIT_BYTES%"=="2" (set "TARGET_PROFILE=u16") else (set "TARGET_PROFILE=u8")
echo [GMP] Starting Ethernet %GMP_DATALINK_TRANSPORT% Data Link Studio, %TARGET_PROFILE% target profile.
pushd "%~dp0"
if errorlevel 1 exit /b 1
python "%~dp0main.py"
set "RESULT=%ERRORLEVEL%"
popd

if "%RESULT%"=="0" exit /b 0
echo [ERROR] Ethernet Data Link Studio exited with code %RESULT%.
pause
exit /b %RESULT%
