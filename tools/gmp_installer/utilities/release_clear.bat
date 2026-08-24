@echo off
setlocal EnableExtensions

call "%~dp0..\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
)

pushd "%GMP_PRO_LOCATION%"
if errorlevel 1 (
    echo [ERROR] Cannot enter the GMP repository root: %GMP_PRO_LOCATION%
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
)

set "RESULT=0"
for /f "delims=" %%d in ('dir /b /s /ad .github 2^>nul') do (
    if exist "%%d" (
        echo remove: "%%d"
        rd /s /q "%%d"
        if errorlevel 1 set "RESULT=1"
    )
)

popd

echo.
if "%RESULT%"=="0" (
    echo Operation complete.
) else (
    echo Operation completed with one or more removal failures.
)
echo.
if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
endlocal & exit /b %RESULT%
