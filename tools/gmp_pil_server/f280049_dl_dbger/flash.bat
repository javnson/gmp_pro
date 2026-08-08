@echo off
setlocal

if not defined CCS_ROOT set "CCS_ROOT=C:\ti\ccs1281\ccs"

set "DSLITE=%CCS_ROOT%\ccs_base\DebugServer\bin\DSLite.exe"
set "PROJECT_DIR=%~dp0"
set "TARGET_CONFIG=%PROJECT_DIR%targetConfigs\TMS320F280049C_LaunchPad.ccxml"
set "FIRMWARE=%PROJECT_DIR%CPU1_LAUNCHXL_FLASH\f280049_dl_dbger.out"

if not exist "%DSLITE%" (
    echo [ERROR] DSLite was not found at "%CCS_ROOT%".
    exit /b 1
)
if not exist "%FIRMWARE%" (
    echo [ERROR] Firmware was not found. Run build_ccs.bat first.
    exit /b 1
)

"%DSLITE%" load --config="%TARGET_CONFIG%" --timeout=120 "%FIRMWARE%"
if errorlevel 1 exit /b 1

echo [SUCCESS] The F280049C firmware is running from the debugger load.
exit /b 0
