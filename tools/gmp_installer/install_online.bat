@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run register_gmp_root.bat first.
    exit /b 1
)
set "GMP_ROOT=%GMP_PRO_LOCATION%"
if not exist "%GMP_ROOT%\tools\gmp_installer\environment_manager.py" (
    echo [ERROR] GMP_PRO_LOCATION does not point to a valid GMP Pro repository.
    exit /b 1
)
set "GMP_BIN=%GMP_ROOT%\bin"
set "PYTHON_HOME=%GMP_BIN%\python"
set "PYTHON_EXE=%PYTHON_HOME%\python.exe"
set "PYTHON_VERSION=3.12.10"
set "PYTHON_INSTALLER=%GMP_BIN%\cache\downloads\python-%PYTHON_VERSION%-amd64.exe"
set "PYTHON_URL=https://www.python.org/ftp/python/%PYTHON_VERSION%/python-%PYTHON_VERSION%-amd64.exe"

echo ========================================================
echo       GMP Pro Private Environment - Online Install
echo ========================================================
echo Root: %GMP_ROOT%
echo Bin : %GMP_BIN%
echo.

if /i "%~1"=="--plan" (
    echo Python       : %PYTHON_VERSION% in bin\python
    echo Applications : portable Git, CMake, Ninja, Doxygen, Graphviz
    echo vcpkg         : repository-local bin\vcpkg and shared bin\vcpkg_installed
    echo Visual Studio : optional; enables suite simulation package restore
    echo Python packages: tools\gmp_installer\requirements-gmp.txt
    echo Repository    : CCS registration, source/SDPE tools, and project .gitignore distribution
    exit /b 0
)

del /q "%GMP_BIN%\gmp_virtual_env_installed.flag" >nul 2>&1

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_proxy.bat"
if errorlevel 1 exit /b 1

if not exist "%PYTHON_EXE%" (
    where curl.exe >nul 2>&1
    if errorlevel 1 (
        echo [ERROR] Windows curl.exe is required for the initial Python download.
        echo         Download %PYTHON_URL% manually to:
        echo         %PYTHON_INSTALLER%
        exit /b 1
    )

    if not exist "%GMP_BIN%\cache\downloads" mkdir "%GMP_BIN%\cache\downloads"
    if not exist "%PYTHON_INSTALLER%" (
        echo [DOWNLOAD] Python %PYTHON_VERSION%...
        curl.exe --fail --location --retry 3 --output "%PYTHON_INSTALLER%.part" "%PYTHON_URL%"
        if errorlevel 1 (
            del /q "%PYTHON_INSTALLER%.part" >nul 2>&1
            echo [ERROR] Python download failed. Existing HTTP_PROXY/HTTPS_PROXY variables are honored.
            exit /b 1
        )
        move /y "%PYTHON_INSTALLER%.part" "%PYTHON_INSTALLER%" >nul
    )

    echo [INSTALL] Installing repository-private Python...
    start /wait "" "%PYTHON_INSTALLER%" /quiet InstallAllUsers=0 TargetDir="%PYTHON_HOME%" PrependPath=0 AppendPath=0 Include_launcher=0 Include_doc=0 Include_test=0 Include_pip=1 Include_tcltk=1 Include_tools=1 Include_symbols=0 Include_debug=0 Shortcuts=0 AssociateFiles=0
    if errorlevel 1 (
        echo [ERROR] Private Python installation failed.
        exit /b 1
    )
)

if not exist "%PYTHON_EXE%" (
    echo [ERROR] Private Python executable was not created: %PYTHON_EXE%
    exit /b 1
)

"%PYTHON_EXE%" "%~dp0environment_manager.py" online %*
exit /b %ERRORLEVEL%
