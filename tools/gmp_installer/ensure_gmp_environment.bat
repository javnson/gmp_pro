@echo off

if not defined GMP_PRO_LOCATION goto :MISSING_ROOT

for %%I in ("%GMP_PRO_LOCATION%") do set "GMP_ENV_GUARD_ROOT=%%~fI"
if not exist "%GMP_ENV_GUARD_ROOT%\tools\gmp_installer\activate_env.bat" goto :INVALID_ROOT

if /i "%GMP_ENV_INSTALLING%"=="%GMP_ENV_GUARD_ROOT%" if /i "%GMP_ENV_ACTIVE%"=="%GMP_ENV_GUARD_ROOT%" goto :SUCCESS

set "GMP_ENV_GUARD_STATE=%GMP_ENV_GUARD_ROOT%\bin\gmp_virtual_env_installed.flag"
set "GMP_ENV_GUARD_PYTHON=%GMP_ENV_GUARD_ROOT%\bin\python\python.exe"
if exist "%GMP_ENV_GUARD_STATE%" goto :ACTIVATE_VIRTUAL
if exist "%GMP_ENV_GUARD_ROOT%\bin\gmp_system_env_installed.flag" goto :ACTIVATE_SYSTEM
if /i "%GMP_ENV_ACTIVE%"=="%GMP_ENV_GUARD_ROOT%" goto :INCOMPLETE
if /i "%GMP_SYSTEM_ROOT%"=="%GMP_ENV_GUARD_ROOT%" goto :INCOMPLETE
goto :USE_SYSTEM

:ACTIVATE_VIRTUAL
set "GMP_ENV_GUARD_MODE=virtual"
set "GMP_ENV_GUARD_ARGUMENT="
goto :ACTIVATE

:ACTIVATE_SYSTEM
set "GMP_ENV_GUARD_MODE=system"
set "GMP_ENV_GUARD_ARGUMENT=--system"

:ACTIVATE
if not exist "%GMP_ENV_GUARD_PYTHON%" goto :CORRUPT_VIRTUAL
if /i "%GMP_ENV_ACTIVE%"=="%GMP_ENV_GUARD_ROOT%" if /i "%GMP_ENV_MODE%"=="%GMP_ENV_GUARD_MODE%" goto :SUCCESS
set "GMP_ENV_GUARD_CWD=%CD%"
call "%GMP_ENV_GUARD_ROOT%\tools\gmp_installer\activate_env.bat" %GMP_ENV_GUARD_ARGUMENT%
set "GMP_ENV_GUARD_RESULT=%ERRORLEVEL%"
cd /d "%GMP_ENV_GUARD_CWD%"
if not "%GMP_ENV_GUARD_RESULT%"=="0" goto :ACTIVATION_FAILED
goto :SUCCESS

:USE_SYSTEM
if exist "%GMP_ENV_GUARD_PYTHON%" echo [WARN] Private Python exists without a completed GMP environment marker; using the system environment.
if /i not "%GMP_ENV_MODE%"=="system" echo [GMP] No completed virtual environment detected; using the system environment.
set "GMP_ENV_MODE=system"
set "GMP_ENV_ACTIVE="
goto :SUCCESS

:MISSING_ROOT
echo [ERROR] GMP_PRO_LOCATION is not defined.
echo         Run install_gmp.bat or install_gmp_virtual_env.bat first.
exit /b 1

:INVALID_ROOT
echo [ERROR] GMP_PRO_LOCATION does not point to a valid GMP Pro repository:
echo         %GMP_ENV_GUARD_ROOT%
goto :FAILURE

:CORRUPT_VIRTUAL
echo [ERROR] The GMP completion marker exists, but its pinned Python is missing.
echo         Re-run the installer for this environment.
goto :FAILURE

:INCOMPLETE
echo [ERROR] The active GMP environment is incomplete. Run the full installer and open a new terminal.
goto :FAILURE

:ACTIVATION_FAILED
echo [ERROR] Failed to activate the GMP virtual environment.
goto :FAILURE

:SUCCESS
set "GMP_ENV_GUARD_ROOT="
set "GMP_ENV_GUARD_STATE="
set "GMP_ENV_GUARD_PYTHON="
set "GMP_ENV_GUARD_CWD="
set "GMP_ENV_GUARD_RESULT="
set "GMP_ENV_GUARD_MODE="
set "GMP_ENV_GUARD_ARGUMENT="
exit /b 0

:FAILURE
set "GMP_ENV_GUARD_ROOT="
set "GMP_ENV_GUARD_STATE="
set "GMP_ENV_GUARD_PYTHON="
set "GMP_ENV_GUARD_CWD="
set "GMP_ENV_GUARD_RESULT="
set "GMP_ENV_GUARD_MODE="
set "GMP_ENV_GUARD_ARGUMENT="
exit /b 1
