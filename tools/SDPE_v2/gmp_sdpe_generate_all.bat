@echo off
setlocal

cd /d "%~dp0"

set "SDPE_LIBRARY=%~1"
if "%SDPE_LIBRARY%"=="" set "SDPE_LIBRARY=examples"

set "SDPE_OUT=%~2"
if "%SDPE_OUT%"=="" set "SDPE_OUT=build"

echo [SDPE] Library: %SDPE_LIBRARY%
echo [SDPE] Output : %SDPE_OUT%

python "%~dp0sdpe.py" --library "%SDPE_LIBRARY%" validate
if errorlevel 1 exit /b %errorlevel%

python "%~dp0sdpe.py" --library "%SDPE_LIBRARY%" generate-all --out "%SDPE_OUT%" --include-prefix ctl/component
exit /b %errorlevel%
