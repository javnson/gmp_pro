@echo off
setlocal

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

cd /d "%GMP_PRO_LOCATION%\ctl\hardware_preset"

echo =======================================================
echo [SDPE] Generating global hardware preset headers
echo =======================================================
echo [SDPE] Source: %GMP_PRO_LOCATION%\ctl\hardware_preset\sdpe_src
echo [SDPE] Output: %GMP_PRO_LOCATION%\ctl\hardware_preset

call "%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_generate_all.bat" "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
exit /b %errorlevel%
