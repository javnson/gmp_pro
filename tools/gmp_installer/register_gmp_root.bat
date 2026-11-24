@echo off

for %%I in ("%~dp0..\..") do set "GMP_CANDIDATE_ROOT=%%~fI"

if not "%GMP_CANDIDATE_ROOT%"=="%GMP_CANDIDATE_ROOT: =%" (
    echo [ERROR] Spaces are present in the GMP repository path:
    echo         %GMP_CANDIDATE_ROOT%
    exit /b 1
)

echo(%GMP_CANDIDATE_ROOT%| %SystemRoot%\System32\findstr.exe /r "[^ -~]" >nul
if not errorlevel 1 (
    echo [ERROR] Non-ASCII characters are present in the GMP repository path:
    echo         %GMP_CANDIDATE_ROOT%
    exit /b 1
)

if /i "%~1"=="--process-only" goto :REGISTER_PROCESS

echo [GMP] Registering GMP_PRO_LOCATION=%GMP_CANDIDATE_ROOT%
%SystemRoot%\System32\setx.exe GMP_PRO_LOCATION "%GMP_CANDIDATE_ROOT%" >nul
if errorlevel 1 (
    echo [ERROR] Failed to write the user GMP_PRO_LOCATION environment variable.
    exit /b 1
)

:REGISTER_PROCESS
set "GMP_PRO_LOCATION=%GMP_CANDIDATE_ROOT%"
set "GMP_CANDIDATE_ROOT="
if /i "%~1"=="--process-only" exit /b 0
echo [GMP] GMP_PRO_LOCATION is available to this installer and future processes.
exit /b 0
