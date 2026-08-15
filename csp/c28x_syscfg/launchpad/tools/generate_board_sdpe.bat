@echo off
setlocal EnableExtensions

set "GMP_BOARD=%~1"
set "GMP_PRO_LOCATION=%~2"
set "GMP_PROJECT_ROOT=%~dp0.."
for %%I in ("%GMP_PROJECT_ROOT%") do set "GMP_PROJECT_ROOT=%%~fI"

if not defined GMP_BOARD (
    echo [ERROR] Board name is required.
    exit /b 1
)
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP repository root is required.
    exit /b 1
)

set "SDPE=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py"
set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
set "REQUIREMENT=%GMP_PROJECT_ROOT%\src\sdpe_mgr\requirements\%GMP_BOARD%\sdpe_requirement.json"
set "BOARD_ROOT=%GMP_PROJECT_ROOT%\C2000Lib_%GMP_BOARD%"

if not exist "%SDPE%" (
    echo [ERROR] SDPE entry point not found: %SDPE%
    exit /b 1
)
if not exist "%REQUIREMENT%" (
    echo [ERROR] Board requirement not found: %REQUIREMENT%
    exit /b 1
)
if not exist "%BOARD_ROOT%" (
    echo [ERROR] Board output directory not found: %BOARD_ROOT%
    exit /b 1
)

python "%SDPE%" --settings "%SDPE_SETTINGS%" validate
if errorlevel 1 exit /b %errorlevel%

python "%SDPE%" --settings "%SDPE_SETTINGS%" generate-project-local "%REQUIREMENT%" --project-dir "%BOARD_ROOT%" --out "%BOARD_ROOT%"
if errorlevel 1 exit /b %errorlevel%

python "%SDPE%" --settings "%SDPE_SETTINGS%" generate-project-matlab-local "%REQUIREMENT%" --project-dir "%GMP_PROJECT_ROOT%\src\sdpe_mgr" --out "%GMP_PROJECT_ROOT%\src\sdpe_mgr\requirements\%GMP_BOARD%"
if errorlevel 1 exit /b %errorlevel%

if not exist "%BOARD_ROOT%\launchpad_board.h" (
    echo [ERROR] SDPE did not produce %BOARD_ROOT%\launchpad_board.h.
    exit /b 1
)

echo [SDPE] Generated %GMP_BOARD% bindings from src\sdpe_mgr\requirements\%GMP_BOARD%.
exit /b 0
