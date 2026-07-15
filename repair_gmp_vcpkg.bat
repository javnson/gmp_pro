@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    pause
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_private_proxy.bat"
if errorlevel 1 (
    echo [GMP] GMP proxy configuration failed.
    pause
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\activate_env.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

"%GMP_PRO_LOCATION%\bin\python\python.exe" "%GMP_PRO_LOCATION%\tools\gmp_installer\environment_manager.py" restore-vcpkg
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo [GMP] vcpkg repair failed with exit code %RESULT%.
    pause
)
exit /b %RESULT%
