@echo off
setlocal

if not defined CCS_ROOT set "CCS_ROOT=C:\ti\ccs1281\ccs"

set "CCS_ECLIPSE=%CCS_ROOT%\eclipse\eclipsec.exe"
for %%I in ("%~dp0.") do set "PROJECT_DIR=%%~fI"
set "CCS_WORKSPACE=%TEMP%\gmp_f280049_dl_ccs_workspace"

if not exist "%CCS_ECLIPSE%" (
    echo [ERROR] CCS was not found at "%CCS_ROOT%".
    echo         Set CCS_ROOT to the CCS installation directory and retry.
    exit /b 1
)

"%CCS_ECLIPSE%" -noSplash -data "%CCS_WORKSPACE%" ^
    -application com.ti.ccstudio.apps.importProject ^
    "-ccs.location" "%PROJECT_DIR%" "-ccs.overwrite"
if errorlevel 1 exit /b 1

"%CCS_ECLIPSE%" -noSplash -data "%CCS_WORKSPACE%" ^
    -application com.ti.ccstudio.apps.buildProject ^
    "-ccs.projects" f280049_dl_dbger ^
    "-ccs.configuration" CPU1_LAUNCHXL_FLASH ^
    "-ccs.buildType" full "-ccs.listProblems"
if errorlevel 1 exit /b 1

echo [SUCCESS] Firmware: %PROJECT_DIR%\CPU1_LAUNCHXL_FLASH\f280049_dl_dbger.out
exit /b 0
