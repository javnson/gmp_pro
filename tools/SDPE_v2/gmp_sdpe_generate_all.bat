@echo off
setlocal

cd /d "%~dp0"

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

set "SDPE_SETTINGS=%~1"
if "%SDPE_SETTINGS%"=="" set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"

echo [SDPE] Settings: %SDPE_SETTINGS%
echo [SDPE] Source  : %GMP_PRO_LOCATION%\ctl\hardware_preset\sdpe_src
echo [SDPE] Output  : %GMP_PRO_LOCATION%\ctl\hardware_preset

python "%~dp0sdpe.py" --settings "%SDPE_SETTINGS%" validate
if errorlevel 1 exit /b %errorlevel%

python "%~dp0sdpe.py" --settings "%SDPE_SETTINGS%" generate-global-hardware
exit /b %errorlevel%
