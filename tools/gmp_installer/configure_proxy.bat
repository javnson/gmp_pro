@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "GMP_PROXY_SOURCE="
set "GMP_PROXY_CANDIDATE="
set "GMP_PROXY_CONFIRMED=0"

if defined GMP_INSTALLER_PROXY_URL (
    set "GMP_PROXY_CANDIDATE=!GMP_INSTALLER_PROXY_URL!"
    set "GMP_PROXY_SOURCE=GMP_INSTALLER_PROXY_URL"
) else if defined HTTPS_PROXY (
    set "GMP_PROXY_CANDIDATE=!HTTPS_PROXY!"
    set "GMP_PROXY_SOURCE=HTTPS_PROXY environment variable"
) else if defined HTTP_PROXY (
    set "GMP_PROXY_CANDIDATE=!HTTP_PROXY!"
    set "GMP_PROXY_SOURCE=HTTP_PROXY environment variable"
) else if defined ALL_PROXY (
    set "GMP_PROXY_CANDIDATE=!ALL_PROXY!"
    set "GMP_PROXY_SOURCE=ALL_PROXY environment variable"
)

if not defined GMP_PROXY_CANDIDATE (
    set "GMP_WINDOWS_PROXY_ENABLED="
    for /f "tokens=3" %%A in ('%SystemRoot%\System32\reg.exe query "HKCU\Software\Microsoft\Windows\CurrentVersion\Internet Settings" /v ProxyEnable 2^>nul') do set "GMP_WINDOWS_PROXY_ENABLED=%%A"
    if /i "!GMP_WINDOWS_PROXY_ENABLED!"=="0x1" (
        for /f "tokens=2,*" %%A in ('%SystemRoot%\System32\reg.exe query "HKCU\Software\Microsoft\Windows\CurrentVersion\Internet Settings" /v ProxyServer 2^>nul') do set "GMP_WINDOWS_PROXY_RAW=%%B"
        if defined GMP_WINDOWS_PROXY_RAW (
            set "GMP_PROXY_CANDIDATE=!GMP_WINDOWS_PROXY_RAW!"
            set "GMP_PROXY_SOURCE=Windows user proxy settings"

            set "GMP_PROTOCOL_PROXY="
            for %%P in (!GMP_WINDOWS_PROXY_RAW:;= !) do (
                for /f "tokens=1,* delims==" %%K in ("%%~P") do (
                    if /i "%%K"=="https" set "GMP_PROTOCOL_PROXY=%%L"
                    if not defined GMP_PROTOCOL_PROXY if /i "%%K"=="http" set "GMP_PROTOCOL_PROXY=%%L"
                )
            )
            if defined GMP_PROTOCOL_PROXY set "GMP_PROXY_CANDIDATE=!GMP_PROTOCOL_PROXY!"
        )
    )
)

if not defined GMP_PROXY_CANDIDATE (
    if /i "!GMP_INSTALLER_PROXY_CHOICE!"=="N" goto :SKIP_PROXY
    if /i "!GMP_INSTALLER_PROXY_CHOICE!"=="Y" goto :REQUEST_PROXY_URL
    if /i not "!GMP_INSTALLER_PROMPT_FOR_PROXY!"=="1" goto :USE_DIRECT_NO_PROXY

    echo [PROXY] No enabled system proxy was detected.
    %SystemRoot%\System32\choice.exe /c YN /n /m "Use a proxy for GMP downloads? [Y/N]: "
    if errorlevel 2 goto :SKIP_PROXY
    goto :REQUEST_PROXY_URL
)

goto :HAVE_PROXY_CANDIDATE

:REQUEST_PROXY_URL
set "GMP_PROXY_CANDIDATE="
set /p "GMP_PROXY_CANDIDATE=Enter proxy URL (for example http://127.0.0.1:7890): "
if not defined GMP_PROXY_CANDIDATE (
    echo [ERROR] A proxy was requested, but no proxy URL was entered.
    exit /b 1
)
set "GMP_PROXY_SOURCE=manual input"
set "GMP_PROXY_CONFIRMED=1"

:HAVE_PROXY_CANDIDATE

if "!GMP_PROXY_CANDIDATE:://=!"=="!GMP_PROXY_CANDIDATE!" set "GMP_PROXY_CANDIDATE=http://!GMP_PROXY_CANDIDATE!"

echo [PROXY] Detected !GMP_PROXY_SOURCE!: !GMP_PROXY_CANDIDATE!
if "!GMP_PROXY_CONFIRMED!"=="1" goto :USE_PROXY
if /i "!GMP_INSTALLER_PROXY_CHOICE!"=="Y" goto :USE_PROXY
if /i "!GMP_INSTALLER_PROXY_CHOICE!"=="N" goto :SKIP_PROXY

%SystemRoot%\System32\choice.exe /c YN /n /m "Use this proxy for GMP downloads? [Y/N]: "
if errorlevel 2 goto :SKIP_PROXY

:USE_PROXY
echo [PROXY] Proxy enabled for this installation process.
endlocal & set "GMP_INSTALLER_PROXY_MODE=proxy" & set "GMP_INSTALLER_PROXY_URL=%GMP_PROXY_CANDIDATE%" & set "HTTP_PROXY=%GMP_PROXY_CANDIDATE%" & set "HTTPS_PROXY=%GMP_PROXY_CANDIDATE%" & set "ALL_PROXY=%GMP_PROXY_CANDIDATE%" & set "http_proxy=%GMP_PROXY_CANDIDATE%" & set "https_proxy=%GMP_PROXY_CANDIDATE%" & set "all_proxy=%GMP_PROXY_CANDIDATE%" & set "NO_PROXY=localhost,127.0.0.1,::1" & set "no_proxy=localhost,127.0.0.1,::1"
exit /b 0

:USE_DIRECT_NO_PROXY
echo [PROXY] No enabled manual proxy was detected. Using a direct connection.
endlocal & set "GMP_INSTALLER_PROXY_MODE=direct" & set "GMP_INSTALLER_PROXY_URL="
exit /b 0

:SKIP_PROXY
echo [PROXY] Proxy disabled for this installation process.
endlocal & set "GMP_INSTALLER_PROXY_MODE=direct" & set "GMP_INSTALLER_PROXY_URL=" & set "HTTP_PROXY=" & set "HTTPS_PROXY=" & set "ALL_PROXY=" & set "http_proxy=" & set "https_proxy=" & set "all_proxy=" & set "NO_PROXY=*" & set "no_proxy=*"
exit /b 0
