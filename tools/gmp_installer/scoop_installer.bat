@echo off
setlocal EnableDelayedExpansion

title Scoop and vcpkg Auto-Installer (Robust)
echo ========================================================
echo       Dev Environment Setup (Scoop + vcpkg)
echo ========================================================
echo.

:: ==========================================
:: 1. Proxy Configuration
:: ==========================================
echo [SETUP] Please enter your proxy address (e.g., 127.0.0.1:7890)
echo Press ENTER to skip (NOT recommended in CN):
set /p PROXY_ADDR=

if defined PROXY_ADDR (
    echo [INFO] Setting global proxy variables
    set http_proxy=http://!PROXY_ADDR!
    set https_proxy=http://!PROXY_ADDR!
    echo Proxy set to: !PROXY_ADDR!
)

echo.
echo ========================================================
echo [1/3] Checking/Installing Scoop
echo ========================================================

:: Define absolute paths
set "SCOOP_HOME=%USERPROFILE%\scoop"
set "SCOOP_CMD=%SCOOP_HOME%\shims\scoop.cmd"

if exist "%SCOOP_CMD%" (
    echo [INFO] Scoop is already installed.
) else (
    echo [ACTION] Installing Scoop via PowerShell
    powershell -NoProfile -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://get.scoop.sh'))"
    
    if errorlevel 1 (
        echo [ERROR] Failed to install Scoop. Check network/proxy.
        pause
        exit /b 1
    )
)

echo.
echo ========================================================
echo [2/3] Checking Git and vcpkg
echo ========================================================

if not exist "%SCOOP_CMD%" (
    echo [ERROR] Scoop executable not found. Cannot proceed.
    pause
    exit /b 1
)

:: Configure Scoop proxy permanent
if defined PROXY_ADDR (
    call "%SCOOP_CMD%" config proxy !PROXY_ADDR!
)

:: --- NEW: Check if Git exists to avoid Scoop error ---
where git >nul 2>&1
if !errorlevel! EQU 0 (
    echo [INFO] Git is already installed. Skipping installation.
) else (
    echo [ACTION] Git not found. Installing via Scoop
    call "%SCOOP_CMD%" install git
    if !errorlevel! NEQ 0 (
        echo [WARN] Git install returned an error or warning. Continuing attempt
    )
)

:: --- Check vcpkg ---
if exist "%SCOOP_HOME%\apps\vcpkg" (
    echo [INFO] vcpkg is already installed.
) else (
    echo [ACTION] Installing vcpkg (This may take a while)
    call "%SCOOP_CMD%" install vcpkg
    if !errorlevel! NEQ 0 (
        echo [ERROR] Failed to install vcpkg.
        pause
        exit /b 1
    )
)

echo.
echo ========================================================
echo [2.5/3] Installing Build Tools (Ninja, Doxygen, Graphviz)
echo ========================================================

:: --- Install Ninja ---
:: Ninja is a small build system with a focus on speed
if exist "%SCOOP_HOME%\apps\ninja" (
    echo [INFO] Ninja is already installed.
) else (
    echo [ACTION] Installing Ninja...
    call "%SCOOP_CMD%" install ninja
    if !errorlevel! NEQ 0 (
        echo [WARN] Failed to install Ninja. Check network.
    )
)

:: --- Install Doxygen ---
:: Doxygen is the standard tool for generating documentation from annotated C++ sources
if exist "%SCOOP_HOME%\apps\doxygen" (
    echo [INFO] Doxygen is already installed.
) else (
    echo [ACTION] Installing Doxygen...
    call "%SCOOP_CMD%" install doxygen
    if !errorlevel! NEQ 0 (
        echo [WARN] Failed to install Doxygen.
    )
)

:: --- Install Graphviz (Dot) ---
:: Graphviz provides the 'dot' tool required by Doxygen to draw dependency graphs
if exist "%SCOOP_HOME%\apps\graphviz" (
    echo [INFO] Graphviz is already installed.
) else (
    echo [ACTION] Installing Graphviz...
    call "%SCOOP_CMD%" install graphviz
    if !errorlevel! NEQ 0 (
        echo [WARN] Failed to install Graphviz. Doxygen graphs may not work.
    )
)

echo.
echo ========================================================
echo [3/3] Integrating with Visual Studio
echo ========================================================

set "VCPKG_EXE=%SCOOP_HOME%\apps\vcpkg\current\vcpkg.exe"

if exist "%VCPKG_EXE%" (
    echo [ACTION] Performing user-wide integration
    "%VCPKG_EXE%" integrate install
    
    if !errorlevel! EQU 0 (
        echo.
        echo ========================================================
        echo [SUCCESS] Setup Complete!
        echo ========================================================
    ) else (
        echo [ERROR] Integration command failed.
    )
) else (
    echo [ERROR] vcpkg.exe not found at expected path:
    echo %VCPKG_EXE%
)
