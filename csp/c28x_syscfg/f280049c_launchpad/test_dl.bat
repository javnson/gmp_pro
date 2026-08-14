@echo off
setlocal

set "BAUD_RATE=%~1"
set "SERIAL_PORT=%~2"
if not defined BAUD_RATE set "BAUD_RATE=921600"

if defined SERIAL_PORT (
    python "%~dp0smoke_test.py" --baudrate %BAUD_RATE% --port %SERIAL_PORT%
) else (
    python "%~dp0smoke_test.py" --baudrate %BAUD_RATE%
)

exit /b %ERRORLEVEL%
