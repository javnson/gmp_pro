@echo off
setlocal EnableExtensions DisableDelayedExpansion
call "%~dp0register_gmp_root.bat" --process-only
if errorlevel 1 exit /b 1

set "GMP_INSTALLER_PROMPT_FOR_PROXY=0"
if /i "%~1"=="online" set "GMP_INSTALLER_PROMPT_FOR_PROXY=1"
if /i "%~1"=="repair" goto :RUN
for %%A in (%*) do if /i "%%~A"=="--plan" goto :RUN
call "%~dp0configure_proxy.bat"
if errorlevel 1 exit /b 1

:RUN
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0bootstrap.ps1" %*
exit /b %ERRORLEVEL%
