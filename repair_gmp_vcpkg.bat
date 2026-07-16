@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    pause
    exit /b 1
)

if not exist "%GMP_PRO_LOCATION%\bin\python\python.exe" (
    echo [GMP] The private Python environment is not installed.
    pause
    exit /b 1
)

"%GMP_PRO_LOCATION%\bin\python\python.exe" "%GMP_PRO_LOCATION%\tools\gmp_installer\environment_manager.py" check-visual-studio
set "GMP_VS_CHECK_RESULT=%ERRORLEVEL%"
if "%GMP_VS_CHECK_RESULT%"=="2" (
    echo [GMP] Nothing was changed. Install Visual Studio with the Desktop development with C++ workload, then retry.
    pause
    exit /b 2
)
if not "%GMP_VS_CHECK_RESULT%"=="0" (
    echo [GMP] Visual Studio capability detection failed with exit code %GMP_VS_CHECK_RESULT%.
    pause
    exit /b %GMP_VS_CHECK_RESULT%
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
