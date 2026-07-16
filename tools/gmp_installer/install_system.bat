@echo off
setlocal EnableExtensions EnableDelayedExpansion

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run register_gmp_root.bat first.
    exit /b 1
)
if not exist "%GMP_PRO_LOCATION%\tools\gmp_installer\requirements-gmp.txt" (
    echo [ERROR] GMP_PRO_LOCATION does not point to a valid GMP Pro repository.
    exit /b 1
)

echo ========================================================
echo       GMP Pro System Development Environment
echo ========================================================
echo Root: %GMP_PRO_LOCATION%
echo.
echo This compatibility mode installs user-scoped applications with Scoop
echo and enables user-wide vcpkg Visual Studio integration.
echo Visual Studio is optional; suite simulation packages are restored only
echo when the x64 C++ workload is available.
echo.

if /i "%~1"=="--plan" (
    echo Applications : git, python, cmake, ninja, doxygen, graphviz, vcpkg
    echo Python packages: tools\gmp_installer\requirements-gmp.txt
    echo Visual Studio : optional; enables suite simulation package restore
    echo vcpkg projects: ctl\suite\*\project\simulate with vcpkg.json, when VS C++ is available
    echo Integration   : user-wide vcpkg integration, when VS C++ is available
    echo Repository    : CCS registration, source/SDPE tools, and project .gitignore distribution
    exit /b 0
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_proxy.bat"
if errorlevel 1 exit /b 1

set "SCOOP_HOME=%USERPROFILE%\scoop"
set "SCOOP_CMD=%SCOOP_HOME%\shims\scoop.cmd"

if not exist "%SCOOP_CMD%" (
    echo [INSTALL] Scoop...
    if defined GMP_INSTALLER_PROXY_URL (
        powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$client = New-Object System.Net.WebClient; $client.Proxy = New-Object System.Net.WebProxy($env:GMP_INSTALLER_PROXY_URL); iex ($client.DownloadString('https://get.scoop.sh'))"
    ) else (
        powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$client = New-Object System.Net.WebClient; $client.Proxy = [System.Net.GlobalProxySelection]::GetEmptyWebProxy(); iex ($client.DownloadString('https://get.scoop.sh'))"
    )
    if errorlevel 1 (
        echo [ERROR] Scoop installation failed. Existing HTTP_PROXY/HTTPS_PROXY variables are honored.
        exit /b 1
    )
)
if not exist "%SCOOP_CMD%" (
    echo [ERROR] Scoop command was not created at "%SCOOP_CMD%".
    exit /b 1
)

for %%P in (git python cmake ninja doxygen graphviz vcpkg) do (
    if not exist "%SCOOP_HOME%\apps\%%P\current" (
        echo [INSTALL] %%P...
        call "%SCOOP_CMD%" install %%P
        if errorlevel 1 (
            echo [ERROR] Scoop failed to install %%P.
            exit /b 1
        )
    ) else (
        echo [OK] %%P is already managed by Scoop.
    )
)

set "PATH=%SCOOP_HOME%\shims;%PATH%"
where python.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is unavailable after Scoop installation.
    exit /b 1
)

echo [INSTALL] GMP Python packages into the Scoop Python environment...
python -m pip install --upgrade pip
if errorlevel 1 exit /b 1
python -m pip install --requirement "%GMP_PRO_LOCATION%\tools\gmp_installer\requirements-gmp.txt"
if errorlevel 1 exit /b 1
python -m pip check
if errorlevel 1 exit /b 1

where vcpkg.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] vcpkg is unavailable after Scoop installation.
    exit /b 1
)
python "%GMP_PRO_LOCATION%\tools\gmp_installer\environment_manager.py" check-visual-studio >nul 2>&1
set "GMP_VS_CHECK_RESULT=!ERRORLEVEL!"
if "!GMP_VS_CHECK_RESULT!"=="2" goto :SKIP_GMP_VCPKG_PROJECT_RESTORE
if not "!GMP_VS_CHECK_RESULT!"=="0" (
    echo [ERROR] Visual Studio capability detection failed with exit code !GMP_VS_CHECK_RESULT!.
    exit /b !GMP_VS_CHECK_RESULT!
)

echo [INSTALL] Enabling user-wide vcpkg integration for compatibility mode...
vcpkg integrate install --disable-metrics
if errorlevel 1 exit /b 1

set "GMP_VCPKG_PROJECT_LIST=%TEMP%\gmp_vcpkg_projects_%RANDOM%_%RANDOM%.txt"
python "%GMP_PRO_LOCATION%\tools\gmp_installer\environment_manager.py" list-vcpkg-projects >"%GMP_VCPKG_PROJECT_LIST%"
if errorlevel 1 (
    del /q "%GMP_VCPKG_PROJECT_LIST%" >nul 2>&1
    echo [ERROR] Failed to discover GMP vcpkg projects.
    exit /b 1
)

set "GMP_VCPKG_PROJECT_COUNT=0"
for /f "usebackq delims=" %%R in ("%GMP_VCPKG_PROJECT_LIST%") do (
    set /a GMP_VCPKG_PROJECT_COUNT+=1
    set "GMP_VCPKG_PROJECT=%GMP_PRO_LOCATION%\%%R"
    echo [INSTALL] Restoring vcpkg manifest: %%R
    vcpkg install --x-manifest-root="!GMP_VCPKG_PROJECT!" --triplet=x64-windows --disable-metrics
    if errorlevel 1 (
        del /q "%GMP_VCPKG_PROJECT_LIST%" >nul 2>&1
        exit /b 1
    )
)
del /q "%GMP_VCPKG_PROJECT_LIST%" >nul 2>&1
if %GMP_VCPKG_PROJECT_COUNT% equ 0 (
    echo [ERROR] No GMP vcpkg projects were discovered.
    exit /b 1
)
goto :GMP_VCPKG_PROJECT_RESTORE_DONE

:SKIP_GMP_VCPKG_PROJECT_RESTORE
echo [OPTIONAL] Visual Studio x64 C++ tools were not found.
echo            Skipping Visual Studio vcpkg integration and suite package restoration.
echo            Hardware, CCS, Python, source-management, and SDPE tools will still be installed.
echo            Install the VS Desktop development with C++ workload later, then rerun install_gmp.bat.

:GMP_VCPKG_PROJECT_RESTORE_DONE

echo [SETUP] Registering CCS facilities product...
python "%GMP_PRO_LOCATION%\tools\facilities_generator\gmp_fac_install_ccs_product.py"
if errorlevel 1 exit /b 1

echo [SETUP] Generating facility configuration...
python "%GMP_PRO_LOCATION%\tools\facilities_generator\gmp_fac_generate_cfg_json.py"
if errorlevel 1 exit /b 1

echo [SETUP] Distributing source-management tools and generated files...
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_distribute_tools_v3.py"
if errorlevel 1 exit /b 1

echo [SETUP] Distributing project-local SDPE toolchains...
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\distribute_sdpe_mgr.py"
if errorlevel 1 exit /b 1

echo [SETUP] Distributing standalone project .gitignore files...
python "%GMP_PRO_LOCATION%\tools\gmp_installer\distribute_project_gitignores.py"
if errorlevel 1 exit /b 1

echo.
echo [SUCCESS] GMP system development environment is ready.
exit /b 0
